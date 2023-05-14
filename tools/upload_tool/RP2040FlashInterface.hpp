#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "DeviceMap.hpp"

#define FLASH_BASE 0x10000000
#define MAX_FLASH_SIZE (16*1024*1024) // 16 MB Flash size
#define UF2_PAGE_SIZE 256u       // All RP2040 UF2 files are have 256 bytes of data for flashing

namespace UploadTool {

class RP2040UF2Error : public std::runtime_error {
    public:
        RP2040UF2Error(std::string error): std::runtime_error(error) {};
};

class RP2040UF2 {
    public:
        struct RP2040Application {
            bool isBootloader;
            // blAppBase only valid if isBootloader true
            uint32_t blAppBase;

            // Pulled from RP2040 binary info
            std::map<uint,std::vector<std::string>> pins;
            std::vector<uint32_t> clientIds;
            std::map<std::pair<int, uint32_t>, std::pair<std::string, uint>> namedFeatureGroups;
            std::map<std::string, std::vector<std::string>> namedFeatureGroupValues;
            std::string programName, programBuildDate, programVersion, programUrl, programDescription;
            std::string boardType, sdkVersion, boot2Name, deviceIpAddress, agentIpAddress;
            std::vector<std::string> programFeatures, buildAttributes;
            uint32_t binaryStart, binaryEnd, agentPort;
        };

        RP2040UF2(std::ifstream &stream);
        RP2040UF2(const char *filename);

        std::string boardType;
        std::vector<RP2040Application> apps;

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

class RP2040FlashInterface {
    public:
        virtual void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) = 0;
        virtual void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) = 0;
        virtual bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) = 0;
        virtual uint32_t tryGetBootloaderSize() const = 0;
        virtual uint64_t getFlashId() const = 0;
        virtual uint32_t getFlashSize() const = 0;
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

std::string hexWord(uint32_t word);
void dumpInfo(RP2040UF2::RP2040Application &app);
void dumpUF2(RP2040UF2 &uf2);
std::shared_ptr<RP2040Device> selectDevice(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap, std::string &boardType, bool autoSelect);
std::shared_ptr<UploadTool::RP2040FlashInterface> catchInBootDelay(std::vector<std::shared_ptr<RP2040Discovery>> discoverySources, DeviceMap &deviceMap, RP2040UF2 &uf2);
void flashImage(std::shared_ptr<RP2040FlashInterface> interface, RP2040UF2 &uf2, bool isOTA);

// Binary Info
void extractBinaryInfo(RP2040UF2 &uf2, RP2040UF2::RP2040Application &appData, uint32_t base = 0);
uint32_t getBootloaderAppBase(std::shared_ptr<RP2040FlashInterface> itf);

};
