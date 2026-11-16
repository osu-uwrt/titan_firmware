#include "canmore_cpp/CANTransport.hpp"
#include "canmore_cpp/DebugClient.hpp"
#include "canmore_cpp/Discovery.hpp"

#include <unistd.h>

using namespace Canmore;

// ========================================
// CANDiscovery
// ========================================

namespace Canmore {

// Listens for heartbeats on the bus, on whichever backend it is
class CANDiscoveryListener : public CANTransport {
public:
    CANDiscoveryListener(CANDiscovery &discovery, const std::string &canInterface):
        CANTransport(canInterface), discovery(discovery) {
        // Configure filter to listen to all heartbeats
        struct can_filter rfilter[] = { { .can_id = CANMORE_CALC_UTIL_ID_C2A(0, CANMORE_CHAN_HEARTBEAT),
                                          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG |
                                                       CANMORE_CALC_FILTER_MASK(false, true, true, true)) } };
        setRxFilters(std::span<can_filter> { rfilter });
    }

protected:
    void handleFrame(const CANFrame &frame) override { discovery.handleHeartbeat(frame); }

private:
    CANDiscovery &discovery;
};

}  // namespace Canmore

CANDiscovery::CANDiscovery(const std::string &canInterface): canInterface(canInterface) {
    listener = std::make_unique<CANDiscoveryListener>(*this, canInterface);

    // We've set up the transport, ready to start processing heartbeats
    // Discovery closes the fd it's given, so it gets its own copy
    int fd = dup(listener->getPollFd());
    if (fd < 0) {
        throw std::system_error(errno, std::generic_category(), "dup");
    }
    startSocketThread(fd);
}

CANDiscovery::~CANDiscovery() {
    // The thread calls into the listener, so stop it before the listener goes
    stopSocketThread();
}

void CANDiscovery::handleSocketData(int socketFd) {
    (void) socketFd;
    listener->handlePollFd();
}

void CANDiscovery::handleHeartbeat(const CANFrame &frame) {
    if (frame.len != sizeof(canmore_heartbeat_t)) {
        // Unexpected length (probably bad packet, ignore it)
        return;
    }

    canmore_id_t id = { .identifier = frame.id };
    uint8_t clientId = id.pkt_std.client_id;
    canmore_heartbeat_t heartbeatData = { .data = frame.data[0] };

    if (id.pkt_std.client_id == 0) {
        // Ignore client id 0
        return;
    }

    // Create device class depending on reported mode
    std::shared_ptr<Device> device;
    if (heartbeatData.pkt.mode == CANMORE_CONTROL_INTERFACE_MODE_NORMAL) {
        device = std::static_pointer_cast<Device>(std::make_shared<CANNormalDevice>(*this, clientId, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_CONTROL_INTERFACE_MODE_BOOT_DELAY) {
        device = std::static_pointer_cast<Device>(std::make_shared<CANBootDelayDevice>(*this, clientId, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_CONTROL_INTERFACE_MODE_BOOTLOADER) {
        device =
            std::static_pointer_cast<Device>(std::make_shared<CANBootloaderDevice>(*this, clientId, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_CONTROL_INTERFACE_MODE_LINUX) {
        device = std::static_pointer_cast<Device>(std::make_shared<CANLinuxDevice>(*this, clientId, heartbeatData));
    }
    else {
        device = std::make_shared<Device>(canInterface, clientId, heartbeatData);
    }

    reportDiscoveredDevice(device);
}

// ========================================
// CAN Device ID/Mode Switching
// ========================================

uint64_t CANNormalDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
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
            auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
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
    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    auto debugClient = DebugClient(interface);
    debugClient.enterBootloader();

    // Get a discovery context to wait for bootloader
    auto discovery = CANDiscovery::create(canInterface);
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, CAN_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

std::shared_ptr<DebugClient> CANNormalDevice::getClient() {
    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    return std::make_shared<DebugClient>(interface);
}

uint64_t CANBootloaderDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
            RegisterPage mcuCtrlPage(interface, CANMORE_CONTROL_INTERFACE_MODE_BOOTLOADER,
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
            auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
            RegisterPage mcuCtrlPage(interface, CANMORE_CONTROL_INTERFACE_MODE_BOOTLOADER,
                                     CANMORE_BL_MCU_CONTROL_PAGE_NUM);

            uint32_t bl_magic = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET);
            if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
                throw BootloaderError("Unexpected bootloader magic");
            }

            cachedBLVersion = interface->readStringPage(CANMORE_CONTROL_INTERFACE_MODE_BOOTLOADER,
                                                        CANMORE_BL_VERSION_STRING_PAGE_NUM);
            isBLVersionCached = true;
        } catch (RegMappedClientError &e) {
            return "";
        }
    }

    return cachedBLVersion;
}

std::shared_ptr<BootloaderClient> CANBootloaderDevice::getClient() {
    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    return std::make_shared<BootloaderClient>(interface);
}

std::shared_ptr<BootloaderClient> CANBootDelayDevice::enterBootloader() {
    const std::vector<uint8_t> enterBootloaderMagic = CANMORE_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    interface->sendRaw(enterBootloaderMagic);

    // Get a discovery context to wait for bootloader
    auto discovery = CANDiscovery::create(canInterface);
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, CAN_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

std::shared_ptr<BootloaderClient> CANBootDelayDevice::waitForBootloader(int64_t timeoutMs) {
    const std::vector<uint8_t> enterBootloaderMagic = CANMORE_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

    // Get a discovery context to wait for next boot delay
    auto discovery = CANDiscovery::create(canInterface);
    auto bdDev = discovery->waitForDevice<BootDelayDevice>(clientId, timeoutMs);
    if (!bdDev) {
        throw BootloaderError("Failed to discover boot delay device again within timeout");
    }

    // Send the command to enter bootloader
    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    interface->sendRaw(enterBootloaderMagic);

    // Wait for it to come back in bootlodaer mode
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, CAN_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

uint64_t CANLinuxDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
            auto linuxClient = LinuxClient(interface);
            cachedFlashId.doubleword = linuxClient.getFlashId();
            isFlashIdCached = true;
        } catch (RegMappedClientError &e) {
            return 0;
        }
    }

    return cachedFlashId.doubleword;
}

std::shared_ptr<LinuxClient> CANLinuxDevice::getClient() {
    auto interface = RegMappedCANClient::create(canInterface, clientId, CANMORE_CHAN_CONTROL_INTERFACE);
    return std::make_shared<LinuxClient>(interface);
}
