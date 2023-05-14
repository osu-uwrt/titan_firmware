#pragma once

#include <array>
#include <bitset>
#include <memory>
#include <string>
#include <vector>

#include "Canmore.hpp"
#include "RegMappedClient.hpp"

namespace Canmore {

class DebugError : public std::runtime_error {
    public:
        DebugError(const char* msg): std::runtime_error(msg) {};
};

class DebugClient {
    public:
        DebugClient(std::shared_ptr<RegMappedClient> client);

        uint64_t getFlashId() {return cachedFlashID.doubleword;}
        void enterBootloader();

    private:
        union flash_id cachedFlashID;
        std::shared_ptr<RegMappedClient> client;
};

};