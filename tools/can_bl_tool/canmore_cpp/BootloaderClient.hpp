#pragma once

#include <array>
#include <bitset>
#include <memory>
#include <string>
#include <vector>

#include "../RP2040FlashInterface.hpp"
#include "RegMappedClient.hpp"
#include "canmore_titan/bootloader_interface.h"

static_assert(MAX_FLASH_SIZE % CANMORE_BL_FLASH_ERASE_SIZE == 0, "Flash size does not divide evenly into erase size");
static_assert(MAX_FLASH_SIZE % UF2_PAGE_SIZE == 0, "Flash size does not divide evenly into page size");
static_assert(UF2_PAGE_SIZE == CANMORE_BL_FLASH_BUFFER_SIZE, "Flash buffer size does not match uf2 page size");

namespace Canmore {

union flash_id {
    uint64_t doubleword;
    uint32_t word[2];
    uint8_t byte[8];
};

class BootloaderError : public std::runtime_error {
    public:
        BootloaderError(const char* msg): std::runtime_error(msg) {};
};

class BootloaderClient: public UploadTool::RP2040FlashInterface {
    public:
        BootloaderClient(std::shared_ptr<RegMappedClient> client);
        void getVersion(std::string &versionOut);

        // Overidden Functions
        void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) override;
        void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
        bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
        void enableBootloaderOverwrite() override;
        void reboot() override;

        bool shouldWarnOnBootloaderOverwrite() override {return true;}
        uint64_t getFlashId() override {return cachedFlashID.doubleword;}
        uint32_t getFlashSize() override {return cachedFlashSize;}

    private:
        std::shared_ptr<RegMappedClient> client;
        union flash_id cachedFlashID;
        uint32_t cachedFlashSize;
        std::bitset<MAX_FLASH_SIZE/CANMORE_BL_FLASH_ERASE_SIZE> erasedSectors;
        std::bitset<MAX_FLASH_SIZE/CANMORE_BL_FLASH_BUFFER_SIZE> writtenPages;
};

};