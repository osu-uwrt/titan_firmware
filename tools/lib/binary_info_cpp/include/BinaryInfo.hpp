#pragma once

#include "RP2040FlashInterface.hpp"

#include <map>
#include <string>
#include <vector>

namespace BinaryInfo {

class AppImage {
public:
    virtual uint32_t getBaseAddress() = 0;
    virtual size_t getSize() = 0;
    virtual std::array<uint8_t, 256> &getAddress(uint32_t flashAddress) = 0;
};

struct AppInfo {
    AppInfo(): isBootloader(false), blAppBase(0), binaryStart(0), binaryEnd(0), agentPort(0) {}

    bool isBootloader;
    // blAppBase only valid if isBootloader true
    uint32_t blAppBase;

    // Pulled from RP2040 binary info
    std::map<uint, std::vector<std::string>> pins;
    std::vector<uint32_t> clientIds;
    std::map<std::pair<int, uint32_t>, std::pair<std::string, uint>> namedFeatureGroups;
    std::map<std::string, std::vector<std::string>> namedFeatureGroupValues;
    std::string programName, programBuildDate, programVersion, programUrl, programDescription;
    std::string boardType, sdkVersion, boot2Name, deviceIpAddress, agentIpAddress, message;
    std::vector<std::string> programFeatures, buildAttributes;
    uint32_t binaryStart, binaryEnd, agentPort;
};

void reportVersionInfo(std::vector<std::pair<std::string, std::string>> &infoOut, AppInfo &firstApp,
                       AppInfo &nestedApp);
void extractAppInfo(AppImage &uf2, AppInfo &appData, uint32_t base = 0);
void extractAppInfo(RP2040FlashInterface &itf, AppInfo &appData, uint32_t base = 0);
void dumpAppInfo(AppInfo &app);

};  // namespace BinaryInfo
