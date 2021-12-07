#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#include "uploader.h"

#define RESET_INTERFACE_SUBCLASS 0x00
#define RESET_INTERFACE_PROTOCOL 0x01
#define RESET_REQUEST_BOOTSEL 0x01
#define RESET_REQUEST_FLASH 0x02

static int find_reset_interface(libusb_device * device) {
    struct libusb_config_descriptor *config;
    int rc;

    rc = libusb_get_active_config_descriptor(device, &config);
    if (rc) {
        if (verbose)
            fprintf(stderr, "Failed to get usb descriptor: %s\n", libusb_error_name(rc));
        return -1;
    }

    for (int i = 0; i < config->bNumInterfaces; i++){
        const struct libusb_interface * interface = &config->interface[i];
        if (interface->altsetting[0].bInterfaceClass == 0xFF && 
            interface->altsetting[0].bInterfaceSubClass == RESET_INTERFACE_SUBCLASS &&
            interface->altsetting[0].bInterfaceProtocol == RESET_INTERFACE_PROTOCOL) {
            
            return interface->altsetting[0].bInterfaceNumber;
        }
    }

    return -1;
}

static bool rp2040_bootrom_reset(rp2040_device_t* device, bool bootsel, bool picoboot_only) {
    if (bootsel) {
        return reboot_bootsel_to_bootsel(device, (picoboot_only ? 1 : 0));
    } else {
        return picoboot_reboot(device->handle, &device->data.bootrom, 0, SRAM_END, 5) == 0;
    }
}

static bool rp2040_device_reset(rp2040_device_t* device, bool bootsel, bool picoboot_only) {
    int reset_interface = find_reset_interface(libusb_get_device(device->handle));
    if (reset_interface == -1) {
        if (verbose)
            fprintf(stderr, "Failed to find reset interface (might not be enabled on build)\n");
        return false;
    }

    int rc;
    rc = libusb_claim_interface(device->handle, reset_interface);
    if (rc < 0) {
        if (verbose)
            fprintf(stderr, "Error claiming interface: %s\n", libusb_error_name(rc));
        return false;
    }

    int disable_mask = 0;
    if (picoboot_only) {
        disable_mask = 1;  // Disable usb mass storage boot (see pico-sdk reset_usb_boot)
    }

    if (bootsel) {
        rc = libusb_control_transfer(device->handle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                                        RESET_REQUEST_BOOTSEL, disable_mask, reset_interface, NULL, 0, 2000);
    } else {
        rc = libusb_control_transfer(device->handle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                                        RESET_REQUEST_FLASH, 0, reset_interface, NULL, 0, 2000);
    }

    return true;
}

bool rp2040_reset(rp2040_device_t* device, bool bootsel, bool picoboot_only) {
    // Note the device should be considered invalid after this function is called
    bool successful = false;
    if (device->in_bootrom_mode) {
        successful = rp2040_bootrom_reset(device, bootsel, picoboot_only);
    } else {
        successful = rp2040_device_reset(device, bootsel, picoboot_only);
    }
    // Make sure that there's enough time for linux to realize the device is gone
    usleep(50000);
    return successful;
}