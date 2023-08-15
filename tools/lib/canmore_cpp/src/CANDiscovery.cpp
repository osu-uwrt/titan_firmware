#include "canmore_cpp/DebugClient.hpp"
#include "canmore_cpp/Discovery.hpp"

#include "titan/canmore.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using namespace Canmore;

// ========================================
// CANDiscovery
// ========================================

CANDiscovery::CANDiscovery(int ifIndex): ifIndex(ifIndex) {
    // Lookup socket name (to report for each CAN device)
    char nameBuf[IF_NAMESIZE];
    if (if_indextoname(ifIndex, nameBuf) != nameBuf) {
        throw std::system_error(errno, std::generic_category(), "if_indextoname");
    }
    interfaceName.assign(nameBuf);

    // Open socket
    int socketFd;
    if ((socketFd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
    }

    // Bind to requested interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifIndex;

    if (bind(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(socketFd);
        throw std::system_error(errno, std::generic_category(), "CAN bind");
    }

    // Configure filter to listen to all heartbeats
    struct can_filter rfilter[] = { { .can_id = CANMORE_CALC_UTIL_ID_C2A(0, CANMORE_CHAN_HEARTBEAT),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG |
                                                   CANMORE_CALC_FILTER_MASK(false, true, true, true)) } };
    if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        close(socketFd);
        throw std::system_error(errno, std::generic_category(), "CAN setsockopt");
    }

    // We've set up the socket, ready to start processing heartbeats
    startSocketThread(socketFd);
}

void CANDiscovery::handleSocketData(int socketFd) {
    struct can_frame frame;
    ssize_t ret = read(socketFd, &frame, sizeof(frame));
    if (ret != sizeof(frame)) {
        if (errno == EAGAIN) {
            return;  // No data to be received
        }
        else {
            throw std::system_error(errno, std::generic_category(), "CAN read");
        }
    }

    if (frame.can_dlc != sizeof(canmore_heartbeat_t)) {
        // Unexpected length (probably bad packet, ignore it)
        return;
    }

    canmore_id_t id = { .identifier = frame.can_id };
    uint8_t clientId = id.pkt_std.client_id;
    canmore_titan_heartbeat_t heartbeatData = { .data = frame.data[0] };

    if (id.pkt_std.client_id == 0) {
        // Ignore client id 0
        return;
    }

    // Create device class depending on reported mode
    std::shared_ptr<Device> device;
    if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL) {
        device = std::static_pointer_cast<Device>(std::make_shared<CANNormalDevice>(*this, clientId, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOT_DELAY) {
        device = std::static_pointer_cast<Device>(std::make_shared<CANBootDelayDevice>(*this, clientId, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER) {
        device =
            std::static_pointer_cast<Device>(std::make_shared<CANBootloaderDevice>(*this, clientId, heartbeatData));
    }
    else {
        device = std::make_shared<Device>(interfaceName, clientId, heartbeatData);
    }

    reportDiscoveredDevice(device);
}

// ========================================
// CAN Device ID/Mode Switching
// ========================================

uint64_t CANNormalDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
            auto debugClient = DebugClient(interface);
            cachedFlashId.doubleword = debugClient.getFlashId();
            isFlashIdCached = true;
        } catch (RegMappedClientError &e) {
            return 0;
        }
    }

    return cachedFlashId.doubleword;
}

std::string CANNormalDevice::getAppVersion() {
    if (!isAppVersionCached) {
        try {
            auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
            auto debugClient = DebugClient(interface);
            cachedAppVersion = debugClient.getVersion();
            isAppVersionCached = true;
        } catch (RegMappedClientError &e) {
            return "";
        }
    }

    return cachedAppVersion;
}

std::shared_ptr<BootloaderClient> CANNormalDevice::switchToBootloaderMode() {
    auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
    auto debugClient = DebugClient(interface);
    debugClient.enterBootloader();

    // Get a discovery context to wait for bootloader
    auto discovery = CANDiscovery::create(ifIndex);
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, CAN_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

std::shared_ptr<DebugClient> CANNormalDevice::getClient() {
    auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
    return std::make_shared<DebugClient>(interface);
}

uint64_t CANBootloaderDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
            RegisterPage mcuCtrlPage(interface, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER,
                                     CANMORE_BL_MCU_CONTROL_PAGE_NUM);

            uint32_t bl_magic = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET);
            if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
                throw BootloaderError("Unexpected bootloader magic");
            }

            cachedFlashId.word[0] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID);
            cachedFlashId.word[1] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID);
            isFlashIdCached = true;
        } catch (RegMappedClientError &e) {
            return 0;
        }
    }

    return cachedFlashId.doubleword;
}

std::string CANBootloaderDevice::getBLVersion() {
    if (!isBLVersionCached) {
        try {
            auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
            RegisterPage mcuCtrlPage(interface, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER,
                                     CANMORE_BL_MCU_CONTROL_PAGE_NUM);

            uint32_t bl_magic = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET);
            if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
                throw BootloaderError("Unexpected bootloader magic");
            }

            cachedBLVersion = interface->readStringPage(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER,
                                                        CANMORE_BL_VERSION_STRING_PAGE_NUM);
            isBLVersionCached = true;
        } catch (RegMappedClientError &e) {
            return "";
        }
    }

    return cachedBLVersion;
}

std::shared_ptr<BootloaderClient> CANBootloaderDevice::getClient() {
    auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
    return std::make_shared<BootloaderClient>(interface);
}

std::shared_ptr<BootloaderClient> CANBootDelayDevice::enterBootloader() {
    const std::vector<uint8_t> enterBootloaderMagic = CANMORE_TITAN_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

    auto interface = RegMappedCANClient::create(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE);
    interface->sendRaw(enterBootloaderMagic);

    // Get a discovery context to wait for bootloader
    auto discovery = CANDiscovery::create(ifIndex);
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, CAN_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}
