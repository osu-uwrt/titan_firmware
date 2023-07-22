#pragma once

#include <array>
#include <bitset>
#include <memory>
#include <string>
#include <vector>

#include "../RP2040FlashInterface.hpp"
#include "Canmore.hpp"
#include "RegMappedClient.hpp"
#include "titan/canmore.h"

static_assert(UF2_PAGE_SIZE == CANMORE_BL_FLASH_BUFFER_SIZE, "Flash buffer size does not match uf2 page size");

namespace Canmore {

class BootloaderError : public std::runtime_error {
    public:
        BootloaderError(const char* msg): std::runtime_error(msg) {};
};

class BootloaderClient: public UploadTool::RP2040FlashInterface {
    public:
        BootloaderClient(std::shared_ptr<RegMappedClient> client);
        std::string getVersion();

        // Overidden Functions
        void eraseSector(uint32_t addr) override;
        void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) override;
        void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
        bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
        void enableBootloaderOverwrite() override;
        void reboot() override;

        bool shouldWarnOnBootloaderOverwrite() override {return true;}
        uint64_t getFlashId() override {return cachedFlashID.doubleword;}
        uint32_t getFlashSize() override {return cachedFlashSize;}
        uint32_t tryGetBootloaderSize() override {return cachedBootloaderSize;}

    private:
        std::shared_ptr<RegMappedClient> client;
        union flash_id cachedFlashID;
        uint32_t cachedFlashSize;
        uint32_t cachedBootloaderSize;
};

};
