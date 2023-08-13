#pragma once

#include <array>
#include <bitset>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "RP2040FlashInterface.hpp"
#include "BinaryInfo.hpp"
#include "DeviceMap.hpp"

namespace UploadTool {

class RP2040UF2Error : public std::runtime_error {
    public:
        RP2040UF2Error(std::string error): std::runtime_error(error) {};
};

class RP2040UF2: public BinaryInfo::AppImage {
    public:
        RP2040UF2(std::ifstream &stream);
        RP2040UF2(const char *filename);

        std::string boardType;
        std::vector<BinaryInfo::AppInfo> apps;

        std::array<uint8_t, 256>& getBlock(uint32_t blockNum);
        std::array<uint8_t, 256>& getAddress(uint32_t flashAddress);
        std::array<uint8_t, 256>& getFlashOffset(uint32_t flashOffset) {return getAddress(FLASH_BASE + flashOffset);}

        uint32_t getBlockAddress(uint32_t blockNum) {return baseAddress + (blockNum * UF2_PAGE_SIZE);}
        uint32_t getBaseAddress() {return baseAddress;}
        uint32_t getBaseFlashOffset() {return baseAddress - FLASH_BASE;}
        uint32_t getNumBlocks() {return uf2Array.size();}
        size_t getSize() {return uf2Array.size() * UF2_PAGE_SIZE;}

    protected:
        uint32_t baseAddress;
        std::vector<std::array<uint8_t, 256>> uf2Array;

    private:
        void initFromStream(std::ifstream &stream);
        void populateWithBinaryInfo();
};

class RP2040EraseWrapper {
    public:
        RP2040EraseWrapper(std::shared_ptr<RP2040FlashInterface> interface): interface(interface), erasedSectors(0) {}
        void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
            uint32_t sectorNum = (addr - FLASH_BASE) / FLASH_ERASE_SIZE;
            if (!erasedSectors.test(sectorNum)) {
                erasedSectors.set(sectorNum);
                interface->eraseSector((sectorNum * FLASH_ERASE_SIZE) + FLASH_BASE);
            }
            interface->writeBytes(addr, bytes);
        }
    private:
        std::shared_ptr<RP2040FlashInterface> interface;
        std::bitset<MAX_FLASH_SIZE/FLASH_ERASE_SIZE> erasedSectors;
};

// ========================================
// Flash UI
// ========================================

std::string hexWord(uint32_t word);
void dumpInfo(BinaryInfo::AppInfo &app);
void dumpUF2(RP2040UF2 &uf2);
std::shared_ptr<RP2040Device> selectDevice(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap, std::string &boardType, bool autoSelect);
std::shared_ptr<RP2040FlashInterface> catchInBootDelay(std::vector<std::shared_ptr<RP2040Discovery>> discoverySources, DeviceMap &deviceMap, RP2040UF2 &uf2);
bool flashImage(std::shared_ptr<RP2040FlashInterface> interface, RP2040UF2 &uf2, bool isOTA);


};
