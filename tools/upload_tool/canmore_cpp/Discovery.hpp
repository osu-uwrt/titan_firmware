#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

#include <arpa/inet.h>

#include "titan/canmore.h"
#include "BootloaderClient.hpp"

#include "../RP2040FlashInterface.hpp"

namespace Canmore {

// Timeout to use when switching between modes for discovery
#define CAN_MODE_SWITCH_TIMEOUT_MS 1500
#define ETH_MODE_SWITCH_TIMEOUT_MS 5000

// ========================================
// Interface Definitions
// ========================================

class Device: public UploadTool::RP2040Device {
    public:
        Device(std::string interfaceName, uint8_t clientId, canmore_titan_heartbeat_t heartbeatData):
            interfaceName(interfaceName), clientId(clientId), termValid(heartbeatData.pkt.term_valid),
            termEnabled(heartbeatData.pkt.term_enabled), inErrorState(heartbeatData.pkt.error),
            mode(heartbeatData.pkt.mode), discoveryTime(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())) {}

        // Constructor to allow creating Device for time comparisons
        Device(std::chrono::milliseconds time):
            interfaceName(""), clientId(0), termValid(false), termEnabled(false), inErrorState(false),
            mode(0), discoveryTime(time) {}

        virtual ~Device() {}

        // RP2040Device overrides
        virtual std::string getMode() const {return "Unknown";}
        std::string getInterface() const override {return interfaceName + " - Client " + std::to_string(clientId);}
        uint64_t getFlashId() override {return 0;}                      // Just return null flash ID unless its supported
        bool supportsFlashInterface() const override {return false;}    // Unknown mode, don't know how to get it into bootloader
        void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
            // Any additional info to show, in this case which mode the canmore device is in
            infoOut.emplace_back("Mode", this->getMode());
        }

        // Attributes
        const std::string interfaceName;
        const uint8_t clientId;
        const bool termValid;
        const bool termEnabled;
        const bool inErrorState;
        const unsigned int mode;
        const std::chrono::milliseconds discoveryTime;

        // Set operators
        bool operator<(const Device &y) const { return discoveryTime < y.discoveryTime; }
        bool operator==(const Device &y) const { return (interfaceName == y.interfaceName) &&
                                                        (clientId == y.clientId);}
};

struct DeviceCmp {
    bool operator() (std::shared_ptr<Device> a, std::shared_ptr<Device> b) const {
        return *a < *b;
    }
};

class NormalDevice: public Device {
    public:
        using Device::Device;

        virtual std::string getAppVersion() = 0;
        virtual uint64_t getFlashId() = 0;
        virtual std::shared_ptr<BootloaderClient> switchToBootloaderMode() = 0;

        void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
            Device::getAdditionalInfo(infoOut);

            // Show the app version for Normal Mode devices
            infoOut.emplace_back("App Version", this->getAppVersion());
        }

        // RP2040Device overrides
        std::string getMode() const override {return "Application";}
        bool supportsFlashInterface() const override {return true;}
        std::shared_ptr<UploadTool::RP2040FlashInterface> getFlashInterface() override
            {return std::static_pointer_cast<UploadTool::RP2040FlashInterface>(switchToBootloaderMode());}
};

class BootDelayDevice: public Device {
    public:
        using Device::Device;

        virtual std::shared_ptr<BootloaderClient> enterBootloader() = 0;

        // RP2040Device overrides
        std::string getMode() const override {return "Boot Delay";}
        // Note although it can be switched to bootloader in this mode, the delay is too short to allow it to be selected
        // Instead just report it can't switch, and require the command line flag to enter the mode
        bool supportsFlashInterface() const override {return false;}
};

class BootloaderDevice: public Device {
    public:
        using Device::Device;

        virtual std::string getBLVersion() = 0;
        virtual uint64_t getFlashId() = 0;
        virtual std::shared_ptr<BootloaderClient> getClient() = 0;

        void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
            Device::getAdditionalInfo(infoOut);

            // Show the bootloader version for Bootloader Mode devices
            infoOut.emplace_back("Bootloader Version", this->getBLVersion());
        }

        // RP2040Device overrides
        std::string getMode() const override {return "Bootloader";}
        bool supportsFlashInterface() const override {return true;}
        std::shared_ptr<UploadTool::RP2040FlashInterface> getFlashInterface() override
            {return std::static_pointer_cast<UploadTool::RP2040FlashInterface>(getClient());}
};

class Discovery: public UploadTool::RP2040Discovery {
    public:
        ~Discovery();

        void discoverDevices(std::vector<std::shared_ptr<UploadTool::RP2040Device>> &devicesOut) override;

        virtual std::string getInterfaceName() = 0;
        void discoverCanmoreDevices(std::vector<std::shared_ptr<Device>> &devicesOut);
        std::shared_ptr<BootloaderClient> catchDeviceInBootDelay(uint8_t clientId);

        // Timeout in milliseconds or -1 if block indefinitely
        template<typename DeviceType> std::shared_ptr<DeviceType> waitForDevice(uint8_t clientId, int64_t timeoutMs) {
            std::shared_ptr<DeviceType> device;
            std::unique_lock<std::mutex> lock(discoveredDevicesMutex);

            // First check if we can find the device (before entering wait state)
            for (std::shared_ptr<Device> d : discoveredDevices) {
                if (d->clientId == clientId && (device = std::dynamic_pointer_cast<DeviceType>(d)) != nullptr) return device;
            }

            if (timeoutMs == 0) {
                // If no blocking, just return null
                return nullptr;
            }
            else if (timeoutMs > 0) {
                // If a timeout specified, wait for timeout
                discoveredNotify.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&](){
                    for (std::shared_ptr<Device> d : discoveredDevices) {
                        if (d->clientId == clientId && (device = std::dynamic_pointer_cast<DeviceType>(d)) != nullptr) return true;
                    }
                    return false;
                });
            }
            else {
                // Else (-1) wait indefinitely
                discoveredNotify.wait(lock, [&](){
                    for (std::shared_ptr<Device> d : discoveredDevices) {
                        if (d->clientId == clientId && (device = std::dynamic_pointer_cast<DeviceType>(d)) != nullptr) return true;
                    }
                    return false;
                });
            }

            // Note that this will return null if it timed out, as the dynamic cast is the last check before exiting the wait condition
            return device;
        }

        // Callback after select finds data available in the socket
        // This call should do what is required to socketFd to read the data, and report it with
        // Must be overidden by client
        virtual void handleSocketData(int socketFd) = 0;

    protected:
        // The child constructor should use singletons
        Discovery();

        // Called after initializing socketFd with a valid file descriptor (for select)
        void startSocketThread(int socketFd);

        // Reports that a device has been discovered by the receiving socket
        void reportDiscoveredDevice(const std::shared_ptr<Device> &device);

    private:

        // FD for the socket created by discovery
        // This socket is polled, and if data is available, handleSocketData is called
        int socketFd;

        // Utility to prune old discovered devices
        // The caller MUST lock the mutex before calling this function
        void pruneDiscoveredDevices();

        // Wakes up socketThread to handle events
        void notifyEventFd();

        // Entry point for the socket thread
        void socketThread();

        // fd to allow polling alongside socketFd
        int threadEventFd;

        // Thread handling polling
        std::thread thread;

        // Condition variable to wait for a device to appear (for catchDeviceInBootDelay)
        std::condition_variable discoveredNotify;

        // Local copy of discoveredDevices
        std::set<std::shared_ptr<Device>, DeviceCmp> discoveredDevices;

        // Lock protecting discoveredDevices
        std::mutex discoveredDevicesMutex;

        // Flag set during running, cleared to stop the thread
        std::atomic_flag threadStopFlag;
};


// ========================================
// CAN Discovery
// ========================================

struct CANDiscoveryKey
{
    int ifIndex;
    CANDiscoveryKey(int ifIndex): ifIndex(ifIndex) {}
    bool operator==(const CANDiscoveryKey &other) const { return ifIndex == other.ifIndex; }
};

struct CANDiscoveryKeyHasher
{
    std::size_t operator()(const CANDiscoveryKey& k) const {return std::hash<int>()(k.ifIndex);}
};

class CANDiscovery: public Discovery, public SocketSingleton<CANDiscovery, CANDiscoveryKey, CANDiscoveryKeyHasher> {
    public:
        friend class SocketSingleton<CANDiscovery, CANDiscoveryKey, CANDiscoveryKeyHasher>;

        int getInterfaceNum() {return ifIndex;}
        std::string getInterfaceName() override {return interfaceName;}

    protected:
        void handleSocketData(int socketFd) override;

    private:
        CANDiscovery(int ifIndex);
        int ifIndex;
        std::string interfaceName;
};

class CANNormalDevice: public NormalDevice {
    public:
        CANNormalDevice(CANDiscovery &parentInterface, uint8_t clientId, canmore_titan_heartbeat_t heartbeatData):
            NormalDevice(parentInterface.getInterfaceName(), clientId, heartbeatData),
            ifIndex(parentInterface.getInterfaceNum()),
            isFlashIdCached(false), isAppVersionCached(false) {}

        std::string getAppVersion() override;
        uint64_t getFlashId() override;
        std::shared_ptr<BootloaderClient> switchToBootloaderMode() override;

    private:
        int ifIndex;
        union flash_id cachedFlashId;
        bool isFlashIdCached;
        std::string cachedAppVersion;
        bool isAppVersionCached;
};

class CANBootloaderDevice: public BootloaderDevice {
    public:
        CANBootloaderDevice(CANDiscovery &parentInterface, uint8_t clientId, canmore_titan_heartbeat_t heartbeatData):
            BootloaderDevice(parentInterface.getInterfaceName(), clientId, heartbeatData),
            ifIndex(parentInterface.getInterfaceNum()),
            isFlashIdCached(false),
            isBLVersionCached(false) {}

        std::string getBLVersion() override;
        uint64_t getFlashId() override;
        std::shared_ptr<BootloaderClient> getClient() override;

    private:
        int ifIndex;
        union flash_id cachedFlashId;
        bool isFlashIdCached;
        std::string cachedBLVersion;
        bool isBLVersionCached;
};

class CANBootDelayDevice: public BootDelayDevice {
    public:
        CANBootDelayDevice(CANDiscovery &parentInterface, uint8_t clientId, canmore_titan_heartbeat_t heartbeatData):
            BootDelayDevice(parentInterface.getInterfaceName(), clientId, heartbeatData),
            ifIndex(parentInterface.getInterfaceNum()) {}

        std::shared_ptr<BootloaderClient> enterBootloader() override;

    private:
        int ifIndex;
};

// ========================================
// Ethernet Discovery
// ========================================

struct EthernetDiscoveryKey
{
    EthernetDiscoveryKey() {}
    bool operator==(const EthernetDiscoveryKey &other) const { (void)other; return true; }
};

struct EthernetDiscoveryKeyHasher
{
    std::size_t operator()(const EthernetDiscoveryKey& k) const { (void)k; return std::hash<int>()(0);}
};

class EthernetDiscovery: public Discovery, public SocketSingleton<EthernetDiscovery, EthernetDiscoveryKey, EthernetDiscoveryKeyHasher> {
    public:
        friend class SocketSingleton<EthernetDiscovery, EthernetDiscoveryKey, EthernetDiscoveryKeyHasher>;

        std::string getInterfaceName() override {return "UDP";}

    protected:
        void handleSocketData(int socketFd) override;

    private:
        EthernetDiscovery();
};

class EthernetNormalDevice: public NormalDevice {
    public:
        // Use lower byte of IP as client ID
        EthernetNormalDevice(EthernetDiscovery &parentInterface, in_addr devAddr, canmore_titan_heartbeat_t heartbeatData):
            NormalDevice(parentInterface.getInterfaceName(), (devAddr.s_addr >> 24), heartbeatData),
            isFlashIdCached(false), isAppVersionCached(false) {
                this->devAddr.s_addr = devAddr.s_addr;
            }

        std::string getAppVersion() override;
        uint64_t getFlashId() override;
        std::shared_ptr<BootloaderClient> switchToBootloaderMode() override;

    private:
        in_addr devAddr;
        union flash_id cachedFlashId;
        bool isFlashIdCached;
        std::string cachedAppVersion;
        bool isAppVersionCached;
};

class EthernetBootloaderDevice: public BootloaderDevice {
    public:
        EthernetBootloaderDevice(EthernetDiscovery &parentInterface, in_addr devAddr, canmore_titan_heartbeat_t heartbeatData):
            BootloaderDevice(parentInterface.getInterfaceName(), (devAddr.s_addr >> 24), heartbeatData),
            isFlashIdCached(false), isBLVersionCached(false) {
                this->devAddr.s_addr = devAddr.s_addr;
            }

        std::string getBLVersion() override;
        uint64_t getFlashId() override;
        std::shared_ptr<BootloaderClient> getClient() override;

    private:
        in_addr devAddr;
        union flash_id cachedFlashId;
        bool isFlashIdCached;
        std::string cachedBLVersion;
        bool isBLVersionCached;
};

class EthernetBootDelayDevice: public BootDelayDevice {
    public:
        EthernetBootDelayDevice(EthernetDiscovery &parentInterface, in_addr devAddr, canmore_titan_heartbeat_t heartbeatData):
            BootDelayDevice(parentInterface.getInterfaceName(), (devAddr.s_addr >> 24), heartbeatData) {
                this->devAddr.s_addr = devAddr.s_addr;
            }

        std::shared_ptr<BootloaderClient> enterBootloader() override;

    private:
        in_addr devAddr;
};

};
