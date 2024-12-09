#include "pico_usb/USBDiscovery.hpp"

#include "pico/usb_reset_interface.h"

#include <chrono>
#include <thread>

using namespace PicoUSB;

int lookupHexChar(unsigned char c) {
    int val;
    if (c >= '0' && c <= '9') {
        val = c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
        val = c - 'A' + 0xA;
    }
    else if (c >= 'a' && c <= 'f') {
        val = c - 'a' + 0xA;
    }
    else {
        val = -1;
    }
    return val;
}

NormalDevice::NormalDevice(std::shared_ptr<USBDeviceHandle> handle, uint8_t serial_desc_index):
    USBDevice(libusb_get_bus_number(libusb_get_device(handle->handle)),
              libusb_get_device_address(libusb_get_device(handle->handle))),
    handle(handle), flashIdCached(0), resetItf(-1), resetItfClaimed(false) {
    unsigned char serialNumber[sizeof(flashIdCached) * 2 + 2];

    // Attempt to lookup serial number
    ssize_t len = checkLibusbErr(
        libusb_get_string_descriptor_ascii(handle->handle, serial_desc_index, serialNumber, sizeof(serialNumber)));
    if (len == sizeof(flashIdCached) * 2) {
        for (size_t i = 0; i < sizeof(flashIdCached); i++) {
            int nibbleUpper = lookupHexChar(serialNumber[2 * i]);
            int nibbleLower = lookupHexChar(serialNumber[2 * i + 1]);
            if (nibbleLower < 0 || nibbleUpper < 0) {
                // If we got an invalid hex character, invalidate the serial number and quit
                flashIdCached = 0;
                break;
            }

            flashIdCached |= (uint64_t) ((nibbleUpper << 4) | nibbleLower) << (8 * i);
        }
    }

    // Lookup and attempt to claim reset interface
    struct libusb_config_descriptor *config;
    checkLibusbErr(libusb_get_active_config_descriptor(libusb_get_device(handle->handle), &config));

    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *interface = &config->interface[i];
        if (interface->altsetting[0].bInterfaceClass == 0xFF &&
            interface->altsetting[0].bInterfaceSubClass == RESET_INTERFACE_SUBCLASS &&
            interface->altsetting[0].bInterfaceProtocol == RESET_INTERFACE_PROTOCOL) {
            resetItf = interface->altsetting[0].bInterfaceNumber;
        }
    }
}

std::shared_ptr<RP2040BootromInterface> NormalDevice::switchToBootromMode() {
    if (resetItf < 0) {
        throw std::logic_error("Device does not support switching to bootrom mode");
    }

    if (!resetItfClaimed) {
        handle->claimInterface(resetItf);
        resetItfClaimed = true;
    }

    // Disable usb mass storage boot, and only allow picoboot interface (see pico-sdk reset_usb_boot)
    int disable_mask = 1;

    // Send reboot request
    // Don't check for errors since it doesn't respond to this request
    libusb_control_transfer(handle->handle, ((int) LIBUSB_REQUEST_TYPE_CLASS) | ((int) LIBUSB_RECIPIENT_INTERFACE),
                            RESET_REQUEST_BOOTSEL, disable_mask, resetItf, NULL, 0, 500);

    // Wait for the new device to appear
    std::shared_ptr<BootromDevice> newDev = nullptr;
    std::chrono::time_point start = std::chrono::steady_clock::now();

    while (newDev == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        newDev = handle->discoverySrc->rediscoverBootromDevice(handle, getFlashId());

        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
            break;
    }

    // Ensure we actually got the device and return it
    if (newDev == nullptr) {
        throw usb_error("Failed to rediscover RP2040 in bootrom - This could be due to the watchdog timer",
                        LIBUSB_ERROR_OTHER);
    }

    return newDev->getBootromInterface();
}
