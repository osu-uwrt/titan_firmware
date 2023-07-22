#include "DeviceMap.hpp"
#include "json.hpp"

using namespace UploadTool;
using json = nlohmann::json;

static uint64_t hexStringToSerial(std::string serialString) {
    if (serialString.length() != 16) {
        throw std::runtime_error("Invalid Serial Number Length");
    }

    uint64_t serial = 0;
    for (size_t i = 0; i < serialString.length(); i++) {
        uint8_t nibble;
        char hexChar = serialString.at(i);
        if ('0' <= hexChar && hexChar <= '9') {
            nibble = hexChar - '0';
        }
        else if ('A' <= hexChar && hexChar <= 'F') {
            nibble = hexChar - 'A' + 10;
        }
        else {
            throw std::runtime_error("Invalid hex character: " + hexChar);
        }
        serial |= ((uint64_t)nibble) << (4 * (i + (i % 2 ? -1 : 1)));
    }

    return serial;
}

DeviceMap::DeviceMap(std::string mapFile) {
    std::ifstream f(mapFile);
    // Configure to allow comments for my descriptions (ramblings) in the device json file
    auto mapJson = json::parse(f, nullptr, true, true);
    for (auto entry : mapJson) {
        std::string name = entry["name"].get_ref<const json::string_t&>();
        std::string boardType = entry["boardType"].get_ref<const json::string_t&>();
        uint64_t serialNumber = hexStringToSerial(entry["serial"].get_ref<const json::string_t&>());

        if (!devices.insert(DeviceIdentifier(name, boardType, serialNumber)).second) {
            std::runtime_error("Duplicate serial number in device map");
        }
    }
}

DeviceIdentifier DeviceMap::lookupSerial(uint64_t flashId) {
    auto unknownDev = DeviceIdentifier(flashId);
    auto it = devices.find(unknownDev);
    if (it == devices.end()) {
        return unknownDev;
    } else {
        return *it;
    }
}
