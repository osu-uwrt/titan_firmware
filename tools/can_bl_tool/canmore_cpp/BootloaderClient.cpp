#include <cstdio>
#include <iostream>

#include "BootloaderClient.hpp"
#include "CRC32.hpp"
#include "canmore_titan/protocol.h"
#include "canmore_titan/bootloader_interface.h"

using namespace Canmore;

#define EXPECTED_VERSION_MAJOR 1
#define EXPECTED_VERSION_MINOR 0

// Defines pulled from board_version.h
#define VERSION_PROTO 0
#define VERSION_DEV 1
#define VERSION_STABLE 2

BootloaderClient::BootloaderClient(std::shared_ptr<RegMappedClient> client): client(client), erasedSectors(0), writtenPages(0) {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_MCU_CONTROL_PAGE_NUM);
    RegisterPage flashCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    // Make sure we're talking to what we expect to
    uint32_t bl_magic = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET);

    if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
        throw BootloaderError("Unexpected bootloader magic");
    }

    // Check version to warn
    uint32_t version_major = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET);
    uint32_t version_minor = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET);
    uint32_t version_type = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET);

    if (version_major != EXPECTED_VERSION_MAJOR || version_minor != EXPECTED_VERSION_MINOR) {
        printf("Warning! Unexpected version %d.%d - This program expects version %d.%d\n",
               version_major, version_minor, EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR);
    }

    if (version_type == VERSION_PROTO) {
        std::cout << "Warning! Bootloader running prototype firmware! This may not operate as expected" << std::endl;
    } else if (version_type == VERSION_DEV) {
        std::cout << "Note: Target bootloader is a development version\n" << std::endl;
    } else if (version_type != VERSION_STABLE) {
        throw BootloaderError("Invalid Version Type");
    }

    // Read data to cache in class
    cachedFlashID.word[0] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID);
    cachedFlashID.word[1] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID);
    cachedFlashSize = flashCtrlPage.readRegister(CANMORE_BL_FLASH_CONTROL_FLASH_SIZE_OFFSET);

    // Disable bootloader overwrite in case it was enabled previously
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, 0);
}

void BootloaderClient::getVersion(std::string &versionOut) {
    client->readStringPage(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_VERSION_STRING_PAGE_NUM, versionOut);
}

void BootloaderClient::readBytes(uint32_t addr, std::vector<uint8_t> &bytesOut) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize || (addr & CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK) != 0) {
        throw BootloaderError("Invalid Read Address");
    }

    RegisterPage flashCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    std::vector<uint32_t> flashContents;
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_READ);
    client->readArray(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_BUFFER_PAGE_NUM, 0,
                        flashContents, CANMORE_BL_FLASH_BUFFER_SIZE/sizeof(uint32_t));

    bytesOut.resize(CANMORE_BL_FLASH_BUFFER_SIZE);
    for (size_t i = 0; i < flashContents.size(); i++) {
        uint32_t word = flashContents.at(i);
        bytesOut.at((i * 4) + 0) = (word >> 0) & 0xFF;
        bytesOut.at((i * 4) + 1) = (word >> 8) & 0xFF;
        bytesOut.at((i * 4) + 2) = (word >> 16) & 0xFF;
        bytesOut.at((i * 4) + 3) = (word >> 24) & 0xFF;
    }
}

void BootloaderClient::writeBytes(uint32_t addr, std::vector<uint8_t> &bytes) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize || (addr & CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK) != 0) {
        throw BootloaderError("Invalid Write Address");
    }
    if (bytes.size() > CANMORE_BL_FLASH_BUFFER_SIZE) {
        throw BootloaderError("Buffer larger than flash page");
    }

    RegisterPage flashCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    uint32_t sectorNum = (addr - FLASH_BASE) / CANMORE_BL_FLASH_ERASE_SIZE;
    uint32_t pageNum = (addr - FLASH_BASE) / CANMORE_BL_FLASH_BUFFER_SIZE;

    // Make sure the page hasn't been written yet
    if (writtenPages.test(pageNum)) {
        throw BootloaderError("Attempting to write to already written page");
    }
    writtenPages.flip(pageNum);

    // Fill flashContents making sure to handle unaligned write sizes
    std::vector<uint32_t> flashContents(CANMORE_BL_FLASH_BUFFER_SIZE/sizeof(uint32_t), 0);
    size_t bytesSize = bytes.size();
    for (size_t word = 0; (word * 4) < bytesSize; word++) {
        uint32_t value = 0;
        for (int byteOff = 0; byteOff + (word * 4) < bytesSize; byteOff++) {
            value |= bytes.at(byteOff + (word * 4)) << (8 * byteOff);
        }

        flashContents.at(word) = value;
    }

    // First erase if needed
    if (!erasedSectors.test(sectorNum)) {
        erasedSectors.flip(sectorNum);
        flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
        flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_ERASE);
    }

    // Write the data
    client->writeArray(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_BUFFER_PAGE_NUM, 0, flashContents);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_WRITE);
}

bool BootloaderClient::verifyBytes(uint32_t addr, std::vector<uint8_t> &bytes) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize || (addr & CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK) != 0) {
        throw BootloaderError("Invalid Write Address");
    }
    if (bytes.size() > CANMORE_BL_FLASH_BUFFER_SIZE) {
        throw BootloaderError("Buffer larger than flash page");
    }

    RegisterPage flashCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    // Compute CRC
    auto bytesCopy = std::vector<uint8_t>(bytes);
    bytesCopy.resize(CANMORE_BL_FLASH_BUFFER_SIZE, 0);  // Pad with 0s (this is how its written to the device)
    uint32_t crcExpected = crcdetail::compute(bytesCopy.data(), bytesCopy.size());

    // Read CRC from device
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_READ);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_CRC);
    uint32_t crcRead = flashCtrlPage.readRegister(CANMORE_BL_FLASH_CONTROL_CRC_OFFSET);

    return crcExpected == crcRead;
}

void BootloaderClient::enableBootloaderOverwrite() {
    client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_FLASH_CONTROL_PAGE_NUM,
                            CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_VALUE);
}

void BootloaderClient::reboot() {
    client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                            CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET, 1);
}