#pragma once

#include <cstdint>

namespace Canmore {

union flash_id {
    uint64_t doubleword;
    uint32_t word[2];
    uint8_t byte[8];
};

};