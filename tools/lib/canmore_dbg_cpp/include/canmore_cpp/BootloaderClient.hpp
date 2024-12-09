#pragma once

#include "GDBClient.hpp"
#include "RP2040FlashInterface.hpp"
#include "canmore_cpp/Canmore.hpp"
#include "canmore_cpp/RegMappedClient.hpp"

#include "canmore/reg_mapped/interface/bootloader.h"

#include <array>
#include <bitset>
#include <memory>
#include <string>
#include <vector>

static_assert(UF2_PAGE_SIZE == CANMORE_BL_FLASH_BUFFER_SIZE, "Flash buffer size does not match uf2 page size");

namespace Canmore {

class BootloaderError : public CanmoreError {
    using CanmoreError::CanmoreError;
};

class BootloaderClient : public RP2040FlashInterface, public GDBClient {
public:
    BootloaderClient(std::shared_ptr<RegMappedClient> client);
    std::string getVersion();

    // RP2040FlashInterface overrides
    void eraseSector(uint32_t addr) override;
    void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) override;
    void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
    bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) override;
    void enableBootloaderOverwrite() override;
    void reboot() override;

    bool shouldWarnOnBootloaderOverwrite() override { return true; }
    uint64_t getFlashId() override { return cachedFlashID.doubleword; }
    uint32_t getFlashSize() override { return cachedFlashSize; }
    uint32_t tryGetBootloaderSize() override { return cachedBootloaderSize; }

    // GDB Stub Overrides
    uint32_t readMemory(uint32_t addr) override;
    void writeMemory(uint32_t addr, uint32_t data) override;
    uint32_t getGDBStubPC() override;
    uint32_t getGDBStubSP() override;
    uint32_t getGDBStubLR() override;

    // Canmore Functions
    void ping() override;

private:
    std::shared_ptr<RegMappedClient> client;
    union flash_id cachedFlashID;
    uint32_t cachedFlashSize;
    uint32_t cachedBootloaderSize;
};

};  // namespace Canmore
