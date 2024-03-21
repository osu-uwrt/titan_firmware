#include "canmore_cpp/BootloaderClient.hpp"

#include "titan/canmore.h"
#include "titan/canmore/crc32.h"

#include <cstdio>
#include <iostream>

using namespace Canmore;

#define cmd_retry_attempts 3

#define bootloader_itf_mode CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER

#define EXPECTED_VERSION_MAJOR 1
#define EXPECTED_VERSION_MINOR 0

// Defines pulled from board_version.h
#define VERSION_DEBUG 0
#define VERSION_DEV 1
#define VERSION_CLEAN 2
#define VERSION_TAGGED 3

BootloaderClient::BootloaderClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage mcuCtrlPage(client, bootloader_itf_mode, CANMORE_BL_MCU_CONTROL_PAGE_NUM);
    RegisterPage flashCtrlPage(client, bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

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
        printf("Warning! Unexpected version %d.%d - This program expects version %d.%d\n", version_major, version_minor,
               EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR);
    }

    if (version_type == VERSION_DEBUG) {
        std::cout << "Warning! Bootloader running debug firmware! This may not operate as expected" << std::endl;
    }
    else if (version_type == VERSION_DEV) {
        std::cout << "Note: Target bootloader is a development version\n" << std::endl;
    }
    else if (version_type != VERSION_CLEAN && version_type != VERSION_TAGGED) {
        throw BootloaderError("Invalid Version Type");
    }

    // Read data to cache in class
    cachedFlashID.word[0] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID);
    cachedFlashID.word[1] = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID);
    cachedFlashSize = flashCtrlPage.readRegister(CANMORE_BL_FLASH_CONTROL_FLASH_SIZE_OFFSET);
    cachedBootloaderSize = flashCtrlPage.readRegister(CANMORE_BL_FLASH_CONTROL_APP_BASE_OFFSET) - FLASH_BASE;

    // Disable bootloader overwrite in case it was enabled previously
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, 0);
}

std::string BootloaderClient::getVersion() {
    return client->readStringPage(bootloader_itf_mode, CANMORE_BL_VERSION_STRING_PAGE_NUM);
}

void BootloaderClient::readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize ||
        (addr & CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK) != 0) {
        throw std::logic_error("Invalid Read Address");
    }

    RegisterPage flashCtrlPage(client, bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    std::vector<uint32_t> flashContents;
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_READ);
    client->readArray(bootloader_itf_mode, CANMORE_BL_FLASH_BUFFER_PAGE_NUM, 0, flashContents,
                      CANMORE_BL_FLASH_BUFFER_SIZE / sizeof(uint32_t));

    for (size_t i = 0; i < flashContents.size(); i++) {
        uint32_t word = flashContents.at(i);
        bytesOut.at((i * 4) + 0) = (word >> 0) & 0xFF;
        bytesOut.at((i * 4) + 1) = (word >> 8) & 0xFF;
        bytesOut.at((i * 4) + 2) = (word >> 16) & 0xFF;
        bytesOut.at((i * 4) + 3) = (word >> 24) & 0xFF;
    }
}

void BootloaderClient::writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    if (bytes.size() != CANMORE_BL_FLASH_BUFFER_SIZE) {
        throw std::logic_error("Invalid write size");
    }
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize ||
        (addr & CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK) != 0) {
        throw std::logic_error("Invalid Write Address");
    }

    RegisterPage flashCtrlPage(client, bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    // Convert 8-bit array to 32-bit word vector which can be written
    std::vector<uint32_t> flashContents(CANMORE_BL_FLASH_BUFFER_SIZE / sizeof(uint32_t), 0);
    size_t bytesSize = bytes.size();
    for (size_t word = 0; (word * 4) < bytesSize; word++) {
        uint32_t value = 0;
        for (int byteOff = 0; byteOff + (word * 4) < bytesSize && byteOff < 4; byteOff++) {
            value |= bytes.at(byteOff + (word * 4)) << (8 * byteOff);
        }

        flashContents.at(word) = value;
    }

    // Write the data (Trying a few times in the event of a comm error, this can sometimes happen with bad cables)
    int attempts = cmd_retry_attempts;
    bool successful = false;
    while (!successful) {
        try {
            client->writeArray(bootloader_itf_mode, CANMORE_BL_FLASH_BUFFER_PAGE_NUM, 0, flashContents);
            flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
            successful = true;
        } catch (RegMappedClientError &e) {
            attempts--;
            if (attempts == 0) {
                throw;
            }
        }
    }

    // Write the data. Can't retry this, if this fails once, abort
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_WRITE);
}

void BootloaderClient::eraseSector(uint32_t addr) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize ||
        (addr & CANMORE_BL_FLASH_ERASE_ADDR_ALIGN_MASK) != 0) {
        throw std::logic_error("Invalid Write Address");
    }

    RegisterPage flashCtrlPage(client, bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
    flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_ERASE);
}

bool BootloaderClient::verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    if (addr < FLASH_BASE || addr - FLASH_BASE >= cachedFlashSize ||
        (addr & CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK) != 0) {
        throw std::logic_error("Invalid Verify Address");
    }
    if (bytes.size() != CANMORE_BL_FLASH_BUFFER_SIZE) {
        throw std::logic_error("Buffer does not match flash page size");
    }

    RegisterPage flashCtrlPage(client, bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM);

    // Compute CRC
    uint32_t crcExpected = crc32_compute(bytes.data(), bytes.size());

    // Read CRC from device (retrying a few times in case of a bad cable or other comm problem)
    int attempts = cmd_retry_attempts;
    bool successful = false;
    uint32_t crcRead = 0;
    while (!successful) {
        try {
            flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, addr);
            flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_READ);
            flashCtrlPage.writeRegister(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, CANMORE_BL_FLASH_CONTROL_COMMAND_CRC);
            crcRead = flashCtrlPage.readRegister(CANMORE_BL_FLASH_CONTROL_CRC_OFFSET);
            successful = true;
        } catch (RegMappedClientError &e) {
            attempts--;
            if (attempts == 0) {
                throw;
            }
        }
    }

    return crcExpected == crcRead;
}

void BootloaderClient::enableBootloaderOverwrite() {
    client->writeRegister(bootloader_itf_mode, CANMORE_BL_FLASH_CONTROL_PAGE_NUM,
                          CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_VALUE);
}

void BootloaderClient::reboot() {
    client->writeRegister(bootloader_itf_mode, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                          CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET, 1);
}

uint32_t BootloaderClient::readMemory(uint32_t addr) {
    RegisterPage gdbStubPage(client, bootloader_itf_mode, CANMORE_BL_GDB_STUB_PAGE_NUM);
    gdbStubPage.writeRegister(CANMORE_BL_GDB_STUB_READ_WORD_ADDR_OFFSET, addr);
    return gdbStubPage.readRegister(CANMORE_BL_GDB_STUB_MEMORY_DATA_OFFSET);
}

void BootloaderClient::writeMemory(uint32_t addr, uint32_t data) {
    RegisterPage gdbStubPage(client, bootloader_itf_mode, CANMORE_BL_GDB_STUB_PAGE_NUM);
    gdbStubPage.writeRegister(CANMORE_BL_GDB_STUB_MEMORY_DATA_OFFSET, data);
    gdbStubPage.writeRegister(CANMORE_BL_GDB_STUB_WRITE_WORD_ADDR_OFFSET, addr);
}

uint32_t BootloaderClient::getGDBStubPC() {
    RegisterPage gdbStubPage(client, bootloader_itf_mode, CANMORE_BL_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_BL_GDB_STUB_PC_REGISTER_OFFSET);
}

uint32_t BootloaderClient::getGDBStubSP() {
    RegisterPage gdbStubPage(client, bootloader_itf_mode, CANMORE_BL_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_BL_GDB_STUB_SP_REGISTER_OFFSET);
}

uint32_t BootloaderClient::getGDBStubLR() {
    RegisterPage gdbStubPage(client, bootloader_itf_mode, CANMORE_BL_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_BL_GDB_STUB_LR_REGISTER_OFFSET);
}

void BootloaderClient::ping() {
    RegisterPage mcuCtrlPage(client, bootloader_itf_mode, CANMORE_BL_MCU_CONTROL_PAGE_NUM);
    uint32_t bl_magic = mcuCtrlPage.readRegister(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET);

    if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
        throw BootloaderError("Unexpected bootloader magic during ping");
    }
}
