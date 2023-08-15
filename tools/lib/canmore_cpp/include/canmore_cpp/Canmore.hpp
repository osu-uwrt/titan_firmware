#pragma once

#include <cstdint>
#include <stdexcept>

namespace Canmore {

class CanmoreError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

union flash_id {
    uint64_t doubleword;
    uint32_t word[2];
    uint8_t byte[8];
};

};  // namespace Canmore
