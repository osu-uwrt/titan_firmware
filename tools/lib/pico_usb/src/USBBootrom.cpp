#include <iostream>
#include "pico_usb/USBDiscovery.hpp"
#include "pico_usb/flash_getid.h"

#include "DeviceMap.hpp"

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
    flash_info_t flash_data = {0};

    try {
        // Load stub to read flash unique ID and JEDEC ID into end of memory
        // This will be fine to write our stub, since this space is reserved for the boot2 writeout during boot
        if (flash_getid_bin_len > 256) {
            throw std::logic_error("Flash Binary Stub won't fit into space");
        }
        const uint32_t flash_bin_stub_loc = SRAM_END - 256;

        conn.write(flash_bin_stub_loc, flash_getid_bin, flash_getid_bin_len);
        conn.exec(flash_bin_stub_loc);
        conn.enter_cmd_xip();
        conn.read(flash_bin_stub_loc + flash_getid_bin_len, flash_data.data, sizeof(flash_data.data));

        cachedFlashId = flash_data.info.flash_id;

        auto& map = DeviceMap::create();
        auto flashInfo = map.lookupFlashChip(flash_data.info.jedec_id);
        if (flashInfo.isUnknown) {
            std::cerr << "Unknown flash chip ID '" << flashInfo.hexId() << "' - please define in DeviceList.jsonc - Falling back to max capacity" << std::endl;
        }
        cachedFlashSize = flashInfo.capacity;
    }
    catch (picoboot::command_failure &e) {
        cachedFlashId = 0;
        cachedFlashSize = FLASH_END - FLASH_START;
    }
    catch (picoboot::connection_error &e) {
        cachedFlashId = 0;
        cachedFlashSize = FLASH_END - FLASH_START;
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
    return cachedFlashSize;
}
