#pragma once

#include <map>
#include <string>
#include <vector>

#include "RP2040FlashInterface.hpp"

namespace UploadTool {

class RP2040UF2;

struct RP2040Application {
    RP2040Application(): isBootloader(false), blAppBase(0), binaryStart(0), binaryEnd(0), agentPort(0) {}

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

void reportVersionInfo(std::vector<std::pair<std::string, std::string>> &infoOut, RP2040Application& firstApp, RP2040Application& nestedApp);
void extractBinaryInfo(RP2040UF2 &uf2, RP2040Application &appData, uint32_t base = 0);
void extractBinaryInfo(RP2040FlashInterface& itf, RP2040Application &appData, uint32_t base = 0);

};
