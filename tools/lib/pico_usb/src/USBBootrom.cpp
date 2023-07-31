#include "pico_usb/USBDiscovery.hpp"
#include "read_flash.h"

using namespace PicoUSB;

static struct picoboot_itf claimPicobootItf(std::shared_ptr<USBDeviceHandle> handle) {
    struct libusb_config_descriptor *config;
    struct picoboot_itf itf;
    checkLibusbErr(libusb_get_active_config_descriptor(libusb_get_device(handle->handle), &config));

    if (config->bNumInterfaces == 1) {
        itf.interface = 0;
    } else {
        itf.interface = 1;
    }
    if (config->interface[itf.interface].altsetting[0].bInterfaceClass == 0xff &&
        config->interface[itf.interface].altsetting[0].bNumEndpoints == 2) {
        itf.out_ep = config->interface[itf.interface].altsetting[0].endpoint[0].bEndpointAddress;
        itf.in_ep = config->interface[itf.interface].altsetting[0].endpoint[1].bEndpointAddress;
    }
    if (!itf.out_ep || !itf.in_ep || (itf.out_ep & 0x80u) || !(itf.in_ep & 0x80u)) {
        throw usb_error("Could not locate PICOBOOT interface", LIBUSB_ERROR_OTHER);
    }

    handle->claimInterface(itf.interface);

    return itf;
}

RP2040BootromInterface::RP2040BootromInterface(std::shared_ptr<USBDeviceHandle> handle):
        handle(handle), conn(handle->handle, claimPicobootItf(handle)) {

    // Try to find discover the flash ID
    union {
        uint8_t data[8];
        uint64_t doubleword;
    } serial = {0};

    try {
        // TODO: Make this a little nicer
        // Currently just puts a small stub to read the flash ID to 0x20001000
        conn.write(0x20001000, serial.data, sizeof(serial));
        conn.exit_xip();
        conn.write(0x20000000, flash_bin, flash_bin_len);
        conn.exec(0x20000000);
        conn.enter_cmd_xip();
        conn.read(0x20001000, serial.data, sizeof(serial));

        cachedFlashId = serial.doubleword;
    }
    catch (picoboot::command_failure &e) {
        cachedFlashId = 0;
    }
    catch (picoboot::connection_error &e) {
        cachedFlashId = 0;
    }

    // Extract current BinaryInfo on device
    try {
        BinaryInfo::extractAppInfo(*this, firstApp);
        if (firstApp.isBootloader) {
            BinaryInfo::extractAppInfo(*this, nestedApp, firstApp.blAppBase);
        }
    }
    catch (std::exception &e) {}    // Ignore any failures during discovery
    catch (...) {}
}

void RP2040BootromInterface::readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) {
    if (addr < FLASH_START || (addr + bytesOut.size()) > FLASH_END) {
        throw std::logic_error("Invalid Read Address");
    }

    conn.read(addr, bytesOut.begin(), bytesOut.size());
}

void RP2040BootromInterface::writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    if (bytes.size() != PAGE_SIZE) {
        throw std::logic_error("Write array size does not match page size");
    }
    if (addr < FLASH_START || addr >= FLASH_END || (addr & (PAGE_SIZE - 1)) != 0) {
        throw std::logic_error("Invalid Write Address");
    }

    conn.write(addr, bytes.begin(), bytes.size());
}

void RP2040BootromInterface::eraseSector(uint32_t addr) {
    static_assert(FLASH_SECTOR_ERASE_SIZE == FLASH_ERASE_SIZE, "Picoboot erase size definition does not UploadTool definition");
    if (addr < FLASH_START || addr >= FLASH_END || (addr & (FLASH_SECTOR_ERASE_SIZE - 1)) != 0) {
        throw std::logic_error("Invalid Erase Address");
    }

    conn.flash_erase(addr, FLASH_SECTOR_ERASE_SIZE);
}

bool RP2040BootromInterface::verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    std::array<uint8_t, UF2_PAGE_SIZE> readout;
    readBytes(addr, readout);
    return bytes == readout;
}

uint32_t RP2040BootromInterface::tryGetBootloaderSize() {
    return (firstApp.isBootloader ? firstApp.blAppBase - FLASH_BASE : 0);
}

uint64_t RP2040BootromInterface::getFlashId() {
    return cachedFlashId;
}

uint32_t RP2040BootromInterface::getFlashSize() {
    return FLASH_END - FLASH_START;     // TODO: Actually try to figure this out
}
