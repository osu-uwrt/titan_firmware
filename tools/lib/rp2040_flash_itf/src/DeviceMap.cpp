#include "DeviceMap.hpp"
#include "json.hpp"

// Generated using CMakeLists file
// Note symbol names are autogenerated by the objcopy command based on the input filename
// The DeviceList.jsonc file is converted into a linkable object, and then linked into the application
// This allows the file to be embedded in the application, and also recompiled by CMake when it changes
extern char embeddedMapStart[] asm( "_binary_DeviceList_jsonc_start" );
extern char embeddedMapEnd[]   asm( "_binary_DeviceList_jsonc_end" );

using json = nlohmann::json;

static uint64_t hexStringToInt(std::string const& str) {
    uint64_t value = 0;
    for (size_t i = 0; i < str.length(); i++) {
        uint8_t nibble;
        char hexChar = str.at(i);
        if ('0' <= hexChar && hexChar <= '9') {
            nibble = hexChar - '0';
        }
        else if ('A' <= hexChar && hexChar <= 'F') {
            nibble = hexChar - 'A' + 10;
        }
        else {
            throw std::runtime_error("Invalid hex character: " + hexChar);
        }
        value |= ((uint64_t)nibble) << (4 * (i + (i % 2 ? -1 : 1)));
    }
    return value;
}

static uint64_t hexStringToSerial(std::string const& serialString) {
    if (serialString.length() != 16) {
        throw std::runtime_error("Invalid Serial Number Length");
    }

    return hexStringToInt(serialString);
}

static uint64_t hexStringToJedecId(std::string const& jedecIdString) {
    if (jedecIdString.length() != 6) {
        throw std::runtime_error("Invalid Serial Number Length");
    }

    return hexStringToInt(jedecIdString);
}

const DeviceMap& DeviceMap::create() {
    static DeviceMap map;
    return map;
}

DeviceMap::DeviceMap() {
    auto mapJson = json::parse(embeddedMapStart, embeddedMapEnd, nullptr, true, true);

    auto& flashSerialMap = mapJson["serialMap"];
    for (auto& entry : flashSerialMap) {
        std::string name = entry["name"].get_ref<const json::string_t&>();
        std::string boardType = entry["boardType"].get_ref<const json::string_t&>();
        uint64_t serialNumber = hexStringToSerial(entry["serial"].get_ref<const json::string_t&>());

        if (!devices.emplace(name, boardType, serialNumber).second) {
            throw std::runtime_error("Duplicate serial number in device map");
        }
    }

    auto& jedecIdMap = mapJson["jedecIdMap"];
    for (auto& entry : jedecIdMap) {
        std::string name = entry["name"].get_ref<const json::string_t&>();
        uint32_t jedecId = hexStringToJedecId(entry["id"].get_ref<const json::string_t&>());
        uint32_t capacity = entry["capacity"].get_ref<const json::number_unsigned_t&>();

        if (capacity > MAX_FLASH_SIZE) {
            throw std::runtime_error("Flash '" + name + "' has capacity larger than RP2040 flash size.");
        }

        if (!flashChips.emplace(name, jedecId, capacity).second) {
            throw std::runtime_error("Duplicate JEDEC ID in device map");
        }
    }
}

DeviceIdentifier DeviceMap::lookupSerial(uint64_t flashId) const {
    auto unknownDev = DeviceIdentifier(flashId);
    auto it = devices.find(unknownDev);
    if (it == devices.end()) {
        return unknownDev;
    } else {
        return *it;
    }
}

FlashChipInfo DeviceMap::lookupFlashChip(uint32_t jedecId) const {
    auto unknownDev = FlashChipInfo(jedecId);
    auto it = flashChips.find(unknownDev);
    if (it == flashChips.end()) {
        return unknownDev;
    } else {
        return *it;
    }
}
