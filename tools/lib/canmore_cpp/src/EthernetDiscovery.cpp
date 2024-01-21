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
// EthernetDiscovery
// ========================================

EthernetDiscovery::EthernetDiscovery() {
    // Open socket
    int socketFd;
    if ((socketFd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
    }

    int optval = 1;
    if (setsockopt(socketFd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) < 0) {
        throw std::system_error(errno, std::generic_category(), "setsockopt");
    }

    // Bind to receive UDP packets
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr = { 0 };
    addr.sin_port = htons(CANMORE_TITAN_ETH_HEARTBEAT_BROADCAST_PORT);

    if (bind(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(socketFd);
        throw std::system_error(errno, std::generic_category(), "bind");
    }

    // We've set up the socket, ready to start processing heartbeats
    startSocketThread(socketFd);
}

void EthernetDiscovery::handleSocketData(int socketFd) {
    canmore_titan_heartbeat_t heartbeatData;

    struct sockaddr_in recvaddr;
    socklen_t recvaddrlen = sizeof(recvaddr);
    ssize_t ret =
        recvfrom(socketFd, &heartbeatData.data, sizeof(heartbeatData), 0, (sockaddr *) &recvaddr, &recvaddrlen);
    if (ret != sizeof(heartbeatData)) {
        if (errno == EAGAIN) {
            return;  // No data to be received
        }
        else if (errno < 0) {
            throw std::system_error(errno, std::generic_category(), "Ethernet read");
        }
        else {
            return;  // Bad packet (maybe some other garbage?)
        }
    }

    // Create device class depending on reported mode
    std::shared_ptr<Device> device;
    if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL) {
        device = std::static_pointer_cast<Device>(
            std::make_shared<EthernetNormalDevice>(*this, recvaddr.sin_addr, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOT_DELAY) {
        device = std::static_pointer_cast<Device>(
            std::make_shared<EthernetBootDelayDevice>(*this, recvaddr.sin_addr, heartbeatData));
    }
    else if (heartbeatData.pkt.mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER) {
        device = std::static_pointer_cast<Device>(
            std::make_shared<EthernetBootloaderDevice>(*this, recvaddr.sin_addr, heartbeatData));
    }
    else {
        device = std::make_shared<Device>(getInterfaceName(), (recvaddr.sin_addr.s_addr >> 24), heartbeatData);
    }

    reportDiscoveredDevice(device);
}

// ========================================
// Ethernet Device ID/Mode Switching
// ========================================

uint64_t EthernetNormalDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
            auto debugClient = DebugClient(interface);
            cachedFlashId.doubleword = debugClient.getFlashId();
            isFlashIdCached = true;
        } catch (RegMappedClientError &e) {
            return 0;
        }
    }

    return cachedFlashId.doubleword;
}

std::string EthernetNormalDevice::getAppVersion() {
    if (!isAppVersionCached) {
        try {
            auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
            auto debugClient = DebugClient(interface);
            cachedAppVersion = debugClient.getVersion();
            isAppVersionCached = true;
        } catch (RegMappedClientError &e) {
            return "";
        }
    }

    return cachedAppVersion;
}

std::shared_ptr<BootloaderClient> EthernetNormalDevice::switchToBootloaderMode() {
    auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
    auto debugClient = DebugClient(interface);
    debugClient.enterBootloader();

    // Get a discovery context to wait for bootloader
    auto discovery = EthernetDiscovery::create();
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, ETH_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

std::shared_ptr<DebugClient> EthernetNormalDevice::getClient() {
    auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
    return std::make_shared<DebugClient>(interface);
}

uint64_t EthernetBootloaderDevice::getFlashId() {
    if (!isFlashIdCached) {
        try {
            auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
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

std::string EthernetBootloaderDevice::getBLVersion() {
    if (!isBLVersionCached) {
        try {
            auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
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

std::shared_ptr<BootloaderClient> EthernetBootloaderDevice::getClient() {
    auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
    return std::make_shared<BootloaderClient>(interface);
}

std::shared_ptr<BootloaderClient> EthernetBootDelayDevice::enterBootloader() {
    const std::vector<uint8_t> enterBootloaderMagic = CANMORE_TITAN_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

    auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
    interface->sendRaw(enterBootloaderMagic);

    // Get a discovery context to wait for bootloader
    auto discovery = EthernetDiscovery::create();
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, ETH_MODE_SWITCH_TIMEOUT_MS);
    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}

std::shared_ptr<BootloaderClient> EthernetBootDelayDevice::waitForBootloader(int64_t timeoutMs) {
    const std::vector<uint8_t> enterBootloaderMagic = CANMORE_TITAN_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

    // Get a discovery context to wait for next boot delay
    auto discovery = EthernetDiscovery::create();
    auto bdDev = discovery->waitForDevice<BootDelayDevice>(clientId, timeoutMs);
    if (!bdDev) {
        throw BootloaderError("Failed to discover boot delay device again within timeout");
    }

    // Send the command to enter bootloader
    auto interface = RegMappedEthernetClient::create(devAddr, CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT);
    interface->sendRaw(enterBootloaderMagic);

    // Wait for it to come back in bootlodaer mode
    std::shared_ptr<BootloaderDevice> dev =
        discovery->waitForDevice<BootloaderDevice>(clientId, ETH_MODE_SWITCH_TIMEOUT_MS);

    if (!dev) {
        throw BootloaderError("Failed to reconnect to device in bootloader mode");
    }
    return dev->getClient();
}
