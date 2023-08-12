#pragma once

#include "RP2040FlashInterface.hpp"

#include <fstream>
#include <string>
#include <set>

struct DeviceIdentifier {
    friend class DeviceMap;
    public:
        DeviceIdentifier(std::string name, std::string boardType, uint64_t serialNumber):
            isUnknown(false), name(name), boardType(boardType), serialNumber(serialNumber) {}

        const bool isUnknown;
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
        DeviceIdentifier(uint64_t serialNumber): isUnknown(true), name("Unknown Board"), boardType("unknown"), serialNumber(serialNumber) {}
};

struct FlashChipInfo {
    friend class DeviceMap;

    public:
        FlashChipInfo(std::string name, uint32_t id, uint32_t capacity):
            isUnknown(false), name(name), id(id), capacity(capacity) {}

        const bool isUnknown;
        const std::string name;
        const uint32_t id;
        const uint32_t capacity;

        std::string hexId() const {
            const char hexbyte[] = "0123456789ABCDEF";
            std::string hexid(6, '0');
            uint32_t flashid = id;
            for (int i = 0; i < 3; i++) {
                hexid.at(i*2) = hexbyte[(flashid >> 4) & 0xF];
                hexid.at(i*2 + 1) = hexbyte[flashid & 0xF];
                flashid >>= 8;
            }
            return hexid;
        }

        bool operator<(const FlashChipInfo &y) const { return id < y.id; }
        bool operator==(const FlashChipInfo &y) const { return id == y.id;}

    protected:
        FlashChipInfo(uint32_t id): isUnknown(true), name("Unknown Flash Chip"), id(id), capacity(MAX_FLASH_SIZE) {}
};

class DeviceMap {
    public:
        static const DeviceMap& create();
        DeviceIdentifier lookupSerial(uint64_t flashId) const;
        FlashChipInfo lookupFlashChip(uint32_t jedecId) const;

    private:
        DeviceMap();
        std::set<DeviceIdentifier> devices;
        std::set<FlashChipInfo> flashChips;
};
