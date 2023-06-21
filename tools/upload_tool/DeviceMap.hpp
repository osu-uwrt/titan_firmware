#pragma once

#include <fstream>
#include <string>
#include <set>

#include "json.hpp"

namespace UploadTool {

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

struct DeviceIdentifier {
    friend class DeviceMap;
    public:
        DeviceIdentifier(const json & deviceEntry):
            name(deviceEntry["name"].get_ref<const json::string_t&>()),
            boardType(deviceEntry["boardType"].get_ref<const json::string_t&>()),
            serialNumber(hexStringToSerial(deviceEntry["serial"].get_ref<const json::string_t&>())) {}

        const std::string name;
        const std::string boardType;
        const uint64_t serialNumber;

        std::string hexSerialNum() const {
            const char hexbyte[] = "0123456789ABCDEF";
            std::string hexid(16, '0');
            uint64_t id = serialNumber;
            for (int i = 0; i < 8; i++) {
                hexid.at(i*2) = hexbyte[(id >> 4) & 0xF];
                hexid.at(i*2 + 1) = hexbyte[id & 0xF];
                id >>= 8;
            }
            return hexid;
        }

        bool operator<(const DeviceIdentifier &y) const { return serialNumber < y.serialNumber; }
        bool operator==(const DeviceIdentifier &y) const { return serialNumber == y.serialNumber;}

    protected:
        DeviceIdentifier(uint64_t serialNumber): name("Unknown Board"), boardType("unknown"), serialNumber(serialNumber) {}
};

class DeviceMap {
    public:
        DeviceMap(std::string mapFile) {
            std::ifstream f(mapFile);
            // Configure to allow comments for my descriptions (ramblings) in the device json file
            auto mapJson = json::parse(f, nullptr, true, true);
            for (auto entry : mapJson) {
                if (!devices.insert(DeviceIdentifier(entry)).second) {
                    std::runtime_error("Duplicate serial number in device map");
                }
            }
        }

        DeviceIdentifier lookupSerial(uint64_t flashId) {
            auto unknownDev = DeviceIdentifier(flashId);
            auto it = devices.find(unknownDev);
            if (it == devices.end()) {
                return unknownDev;
            } else {
                return *it;
            }
        }

    private:
        std::set<DeviceIdentifier> devices;
};

};