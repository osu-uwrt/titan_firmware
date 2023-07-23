#include <iostream>
#include <sstream>
#include "picoboot_connection_cxx.h"
#include "USBDiscovery.hpp"

using namespace PicoUSB;

#define PICO_VID 0x2e8a
#define PICO_BOOTROM_PID 0x0003
#define PICO_SERIAL_PID 0x000a
#define PICO_PROBE_PID 0x000c

USBDiscovery::USBDiscovery(): ctx(NULL) {
    checkLibusbErr(libusb_init(&ctx));
}

std::shared_ptr<BootromDevice> USBDiscovery::rediscoverBootromDevice(std::shared_ptr<USBDeviceHandle> handle, uint64_t flashId) {
    USBDeviceList devList(ctx);
    uint8_t expectedBusNum = libusb_get_bus_number(libusb_get_device(handle->handle));

    while (devList.hasNext()) {
        libusb_device* entry = devList.next();

        try {
            // When the device re-appears, it will be on the same bus, but with a different address
            // We can reduce searching by restricting it to only devices on the same bus
            if (libusb_get_bus_number(entry) != expectedBusNum) {
                continue;
            }

            // Next make sure that we don't have any other RP2040s with the same bus/address as this one
            // Since linux will not re-use USB device numbers, we can assume that if we've seen it before, it is not the new device
            // Additionally if we try to re-enumerate an existing device, the interface may already be claimed and it will fail to discover
            // Easier to just skip ones we already know

            auto itr = discoveredDevices.begin();
            bool foundMatch = false;
            while (itr != discoveredDevices.end()) {
                auto existingDev = itr->lock();
                if (!existingDev) {
                    // Weak pointer expired, remove it from our discovered list
                    discoveredDevices.erase(itr);
                    continue;
                }
                itr++;

                if (existingDev->busNum == libusb_get_bus_number(entry) && existingDev->devAddr == libusb_get_device_address(entry)) {
                    foundMatch = true;
                }
            }

            if (foundMatch) {
                // We found an existing device with the same address and bus number
                // Just skip
                continue;
            }

            // Finally we will need to check the descriptor and see if it is in fact an RP2040
            struct libusb_device_descriptor desc;
            checkLibusbErr(libusb_get_device_descriptor(entry, &desc));

            if (desc.idVendor == PICO_VID && desc.idProduct == PICO_BOOTROM_PID) {
                auto newHandle = std::make_shared<USBDeviceHandle>(entry, this->create());
                auto dev = std::make_shared<BootromDevice>(newHandle);

                // Ensure that discovered device matches our serial number
                if (dev->getFlashId() == flashId) {
                    discoveredDevices.push_back(dev);
                    return dev;
                }
            }
        }
        // Ignore any exceptions during enumeration
        catch (std::exception &e) {}
        catch (...) {}
    }

    return nullptr;
}

void USBDiscovery::discoverDevices(std::vector<std::shared_ptr<UploadTool::RP2040Device>> &devicesOut) {
    USBDeviceList devList(ctx);

    while (devList.hasNext()) {
        libusb_device* entry = devList.next();

        // First check if we already discovered this device and have a handle for it
        auto itr = discoveredDevices.begin();
        bool foundMatch = false;
        while (itr != discoveredDevices.end()) {
            auto existingDev = itr->lock();
            if (!existingDev) {
                // Weak pointer expired, remove it from our discovered list
                discoveredDevices.erase(itr);
                continue;
            }
            itr++;

            if (existingDev->busNum == libusb_get_bus_number(entry) && existingDev->devAddr == libusb_get_device_address(entry)) {
                devicesOut.push_back(existingDev);
                foundMatch = true;
            }
        }
        if (foundMatch) continue;

        // If not begin discovery
        try {
            struct libusb_device_descriptor desc;
            checkLibusbErr(libusb_get_device_descriptor(entry, &desc));

            if (desc.idVendor == PICO_VID) {
                if (desc.idProduct == PICO_SERIAL_PID) {
                    auto handle = std::make_shared<USBDeviceHandle>(entry, this->create());
                    auto dev = std::make_shared<NormalDevice>(handle, desc.iSerialNumber);
                    devicesOut.push_back(dev);
                    discoveredDevices.push_back(dev);
                }
                else if (desc.idProduct == PICO_BOOTROM_PID) {
                    auto handle = std::make_shared<USBDeviceHandle>(entry, this->create());
                    auto dev = std::make_shared<BootromDevice>(handle);
                    devicesOut.push_back(dev);
                    discoveredDevices.push_back(dev);
                }
                else if (desc.idProduct == PICO_PROBE_PID) {
                    char serialNumber[sizeof(uint64_t) * 2 + 2];
                    {
                        USBDeviceHandle handle(entry, this->create());

                        // Attempt to lookup serial number
                        ssize_t len = checkLibusbErr(libusb_get_string_descriptor_ascii(handle.handle, desc.iSerialNumber, (unsigned char*)serialNumber, sizeof(serialNumber)));
                        if (len != sizeof(uint64_t) * 2) {
                            throw usb_error("Invalid Picoprobe Serial String", LIBUSB_ERROR_OTHER);
                        }
                    }
                    std::string serialNumStr(serialNumber);
                    devicesOut.push_back(std::make_shared<PicoprobeDevice>(serialNumStr));
                }
            }
        }
        catch (usb_error &e) {
            std::cerr << "Error discovering usb device on Bus " << (int) libusb_get_bus_number(entry) << " Addr " << (int) libusb_get_device_address(entry) << std::endl;
            std::cerr << "  what():  " << e.what() << std::endl;
            if (e.code().value() == LIBUSB_ERROR_ACCESS) {
                std::cerr << "Ensure you have permissions to access the USB device" << std::endl;
            }
            std::cerr << std::endl;
        }
        catch (picoboot::command_failure &e) {
            std::cerr << "Error discovering usb device on Bus " << (int) libusb_get_bus_number(entry) << " Addr " << (int) libusb_get_device_address(entry) << std::endl;
            std::cerr << "  Picoboot Command Failure: " << e.what() << std::endl;
            std::cerr << std::endl;
        }
        catch (picoboot::connection_error &e) {
            std::cerr << "Error discovering usb device on Bus " << (int) libusb_get_bus_number(entry) << " Addr " << (int) libusb_get_device_address(entry) << std::endl;
            std::cerr << "  Picoboot Connection Failur: Libusb error " << e.libusb_code << std::endl;
            std::cerr << std::endl;
        }
        catch (PicoprobeError &e) {
            std::cerr << "Error discovering RP2040 via Picoprobe:" << std::endl;
            std::cerr << "  what(): " << e.what() << std::endl;
            std::cerr << std::endl;
        }
    }
}

USBDiscovery::~USBDiscovery() {
    if (ctx) {
        libusb_exit(ctx);
    }
}
