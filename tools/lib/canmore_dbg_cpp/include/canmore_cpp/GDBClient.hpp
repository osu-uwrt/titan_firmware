#pragma once

#include <cstdint>

namespace Canmore {

class GDBClient {
public:
    virtual uint32_t readMemory(uint32_t addr) = 0;
    virtual void writeMemory(uint32_t addr, uint32_t data) = 0;
    virtual uint32_t getGDBStubPC() = 0;
    virtual uint32_t getGDBStubSP() = 0;
    virtual uint32_t getGDBStubLR() = 0;

    virtual void ping() = 0;
};

};  // namespace Canmore
