#pragma once

#include "BinaryInfo.hpp"
#include "PicoprobeClient.hpp"
#include "RP2040FlashInterface.hpp"
#include "picoboot_connection_cxx.h"

#include <libusb.h>
#include <system_error>

namespace PicoUSB {

// ========================================
// Libusb Error Handling
// ========================================

struct libusb_category : std::error_category {
    const char *name() const noexcept override { return "libusb"; }
    std::string message(int ev) const override { return std::to_string(ev) + " (" + libusb_error_name(ev) + ")"; }
};

class usb_error : public std::system_error {
    typedef std::system_error Base;

public:
    usb_error(const std::string &msg, libusb_error error_code): Base(error_code, libusb_category(), msg) {}
};

static inline ssize_t checkLibusbErrFunc(const char *func, ssize_t err) {
    if (err < 0)
        throw usb_error(func, static_cast<libusb_error>(err));
    return err;
}
#define checkLibusbErr(func) checkLibusbErrFunc(#func, func)

// ========================================
// Libusb Datatype Wrappers
// ========================================

class USBDeviceList {
public:
    USBDeviceList(libusb_context *ctx): _list(NULL), _idx(0) {
        _cnt = checkLibusbErr(libusb_get_device_list(ctx, &_list));
    }
    ~USBDeviceList() {
        if (_list)
            libusb_free_device_list(_list, 1);
    }

    bool hasNext() { return _idx < _cnt; }
    libusb_device *next() {
        if (_idx >= _cnt)
            throw std::out_of_range("USBDeviceList next");
        return _list[_idx++];
    }

private:
    libusb_device **_list;
    ssize_t _idx;
    ssize_t _cnt;
};

class USBDiscovery;
class USBDeviceHandle {
public:
    USBDeviceHandle(libusb_device *dev, std::shared_ptr<USBDiscovery> discovery):
        handle(NULL), discoverySrc(discovery) {
        checkLibusbErr(libusb_open(dev, &handle));
    }
    ~USBDeviceHandle() {
        for (int itf : claimed_itfs)
            libusb_release_interface(handle, itf);
        if (handle)
            libusb_close(handle);
    }

    void claimInterface(int itf) {
        checkLibusbErr(libusb_claim_interface(handle, itf));
        claimed_itfs.push_back(itf);
    }

    libusb_device_handle *handle;
    // This holds a handle to the discovery, holding the libusb context.
    // This prevents context destruction while a handle is still around
    std::shared_ptr<USBDiscovery> discoverySrc;

private:
    std::vector<int> claimed_itfs;
};

// ========================================
// Bootrom Interface
// ========================================

class RP2040BootromInterface : public RP2040FlashInterface {
public:
    RP2040BootromInterface(std::shared_ptr<USBDeviceHandle> handle);

    void eraseSector(uint32_t addr) override;
    void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) override;
    void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
    bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
    uint32_t tryGetBootloaderSize() override;
    uint64_t getFlashId() override;
    uint32_t getFlashSize() override;
    void reboot() override;

    bool shouldWarnOnBootloaderOverwrite() override { return false; }

private:
    std::shared_ptr<USBDeviceHandle> handle;
    picoboot::connection conn;
    uint64_t cachedFlashId;
    uint32_t cachedFlashSize;

public:
    BinaryInfo::AppInfo firstApp, nestedApp;
};

// ========================================
// Device Discovery Types
// ========================================

class USBDevice : public RP2040Device {
public:
    USBDevice(uint8_t busNum, uint8_t devAddr): busNum(busNum), devAddr(devAddr) {}

    // RP2040Device overrides
    virtual std::string getMode() const { return "Unknown"; }
    std::string getInterface() const override { return "USB"; }
    bool supportsFlashInterface() const override { return true; }
    void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
        // Any additional info to show, in this case which mode the canmore device is in
        infoOut.emplace_back("Mode", getMode());
    }

    // Uniquely identifies USB device
    const uint8_t busNum;
    const uint8_t devAddr;
};

class NormalDevice : public USBDevice {
public:
    NormalDevice(std::shared_ptr<USBDeviceHandle> handle, uint8_t serial_desc_index);

    std::shared_ptr<RP2040BootromInterface> switchToBootromMode();

    // RP2040Device overrides
    bool supportsFlashInterface() const override { return resetItf >= 0; }
    uint64_t getFlashId() override { return flashIdCached; }
    std::string getMode() const override { return "Application"; }
    std::shared_ptr<RP2040FlashInterface> getFlashInterface() override {
        return std::static_pointer_cast<RP2040FlashInterface>(switchToBootromMode());
    }

private:
    std::shared_ptr<USBDeviceHandle> handle;
    uint64_t flashIdCached;
    int resetItf;
    bool resetItfClaimed;
};

class BootromDevice : public USBDevice {
public:
    BootromDevice(std::shared_ptr<USBDeviceHandle> handle):
        USBDevice(libusb_get_bus_number(libusb_get_device(handle->handle)),
                  libusb_get_device_address(libusb_get_device(handle->handle))),
        itf(new RP2040BootromInterface(handle)) {}

    std::shared_ptr<RP2040BootromInterface> getBootromInterface() { return itf; }

    void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
        USBDevice::getAdditionalInfo(infoOut);
        BinaryInfo::reportVersionInfo(infoOut, itf->firstApp, itf->nestedApp);
    }

    // RP2040Device overrides
    uint64_t getFlashId() { return itf->getFlashId(); }
    std::string getMode() const override { return "Bootrom"; }
    std::shared_ptr<RP2040FlashInterface> getFlashInterface() override {
        return std::static_pointer_cast<RP2040FlashInterface>(getBootromInterface());
    }

private:
    std::shared_ptr<RP2040BootromInterface> itf;
};

// ========================================
// Device Class
// ========================================

class USBDiscovery : public RP2040Discovery {
public:
    ~USBDiscovery();
    static std::shared_ptr<USBDiscovery> create(bool picoprobeOnly = false) {
        static auto inst = std::shared_ptr<USBDiscovery>(new USBDiscovery(picoprobeOnly));
        return inst;
    }

    std::shared_ptr<BootromDevice> rediscoverBootromDevice(std::shared_ptr<USBDeviceHandle> handle, uint64_t flashId);

    void discoverDevices(std::vector<std::shared_ptr<RP2040Device>> &devicesOut) override;

private:
    USBDiscovery(bool picoprobeOnly);
    libusb_context *ctx;
    bool picoprobeOnly;
    std::vector<std::weak_ptr<USBDevice>> discoveredDevices;
};

};  // namespace PicoUSB
