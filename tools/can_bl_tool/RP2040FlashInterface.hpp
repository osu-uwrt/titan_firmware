#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "DeviceMap.hpp"

#define BOOTLOADER_SIZE 0x4000  // TODO: Read this from the binary info
#define FLASH_BASE 0x10000000
#define MAX_FLASH_SIZE (16*1024*1024) // 16 MB Flash size
#define UF2_PAGE_SIZE 256       // All RP2040 UF2 files are have 256 bytes of data for flashing

namespace UploadTool {

class RP2040UF2 {
    public:
        RP2040UF2(std::ifstream &stream, bool isOTA);
        RP2040UF2(const char *filename, bool isOTA);

        const bool isOTA;

        void getBlock(uint32_t blockNum, std::vector<uint8_t> &dataOut);
        void getAddress(uint32_t flashAddress, std::vector<uint8_t> &dataOut);
        void getFlashOffset(uint32_t flashOffset, std::vector<uint8_t> &dataOut) {getAddress(FLASH_BASE + flashOffset, dataOut);}

        uint32_t getBlockAddress(uint32_t blockNum) {return baseAddress + (blockNum * UF2_PAGE_SIZE);}
        uint32_t getBaseAddress() {return baseAddress;}
        uint32_t getBaseFlashOffset() {return baseAddress - FLASH_BASE;}
        uint32_t getNumBlocks() {return uf2Array.size();}
        size_t getSize() {return uf2Array.size() * UF2_PAGE_SIZE;}

    protected:
        uint32_t baseAddress;
        std::vector<std::array<uint8_t, 256>> uf2Array;

    private:
        void initFromStream(std::ifstream &stream, bool isOTA);
};

class RP2040FlashInterface {
    public:
        virtual void readBytes(uint32_t addr, std::vector<uint8_t> &bytesOut) = 0;
        virtual void writeBytes(uint32_t addr, std::vector<uint8_t> &bytes) = 0;
        virtual bool verifyBytes(uint32_t addr, std::vector<uint8_t> &bytes) = 0;
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
        virtual std::string getMode() const = 0;
        virtual std::string getInterface() const = 0;
        virtual uint64_t getFlashId() = 0;
        virtual bool supportsFlashInterface() const = 0;

        virtual std::shared_ptr<RP2040FlashInterface> getFlashInterface() {
            throw std::runtime_error("Device can not enter flashing mode!");
        };
};

class RP2040Discovery {
    public:
        virtual void discoverDevices(std::vector<std::shared_ptr<RP2040Device>> &devicesOut) = 0;
};

// ========================================
// Flash UI
// ========================================

std::shared_ptr<RP2040FlashInterface> selectInterface(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap);
void flashImage(RP2040FlashInterface &interface, RP2040UF2 &uf2);

};
