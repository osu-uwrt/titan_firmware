#pragma once

#include <stdexcept>
#include <array>
#include <bitset>
#include <memory>
#include <string>
#include <vector>

#define FLASH_BASE 0x10000000
#define MAX_FLASH_SIZE (16*1024*1024) // 16 MB Flash size
#define FLASH_ERASE_SIZE 4096u        // All RP2040 flash have 4KB erase size
#define UF2_PAGE_SIZE 256u            // All RP2040 UF2 files are have 256 bytes of data for flashing
static_assert(MAX_FLASH_SIZE % FLASH_ERASE_SIZE == 0, "Flash size does not divide evenly into erase size");
static_assert(MAX_FLASH_SIZE % UF2_PAGE_SIZE == 0, "Flash size does not divide evenly into page size");

class RP2040FlashInterface {
    public:
        virtual void eraseSector(uint32_t addr) = 0;
        virtual void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) = 0;
        virtual void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) = 0;
        virtual bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) = 0;
        virtual uint32_t tryGetBootloaderSize() = 0;
        virtual uint64_t getFlashId() = 0;
        virtual uint32_t getFlashSize() = 0;
        virtual void reboot() = 0;

        // Bootloader overwrite features
        virtual bool shouldWarnOnBootloaderOverwrite() = 0;
        // Should be implemented if shouldWarnOnBootloaderOverwrite is true
        virtual void enableBootloaderOverwrite() {};
};

class RP2040Device {
    public:
        virtual std::string getInterface() const = 0;
        virtual uint64_t getFlashId() = 0;
        virtual bool supportsFlashInterface() const = 0;

        // Device any additional info to show during initialization
        virtual void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) = 0;

        virtual std::shared_ptr<RP2040FlashInterface> getFlashInterface() {
            throw std::runtime_error("Device can not enter flashing mode!");
        };
};

class RP2040Discovery {
    public:
        virtual void discoverDevices(std::vector<std::shared_ptr<RP2040Device>> &devicesOut) = 0;
};
