#pragma once

#include <fstream>
#include <string>
#include <set>

namespace UploadTool {

struct DeviceIdentifier {
    friend class DeviceMap;
    public:
        DeviceIdentifier(std::string name, std::string boardType, uint64_t serialNumber):
            name(name), boardType(boardType), serialNumber(serialNumber) {}

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
        DeviceMap(std::string mapFile);
        DeviceIdentifier lookupSerial(uint64_t flashId);

    private:
        std::set<DeviceIdentifier> devices;
};

};
