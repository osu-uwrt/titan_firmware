#pragma once

#include "canmore_cpp/Canmore.hpp"
#include "canmore_cpp/RegMappedClient.hpp"

namespace Canmore {

class LinuxClientError : public CanmoreError {
    using CanmoreError::CanmoreError;
};

class LinuxClient {
public:
    LinuxClient(std::shared_ptr<RegMappedClient> client);

    std::string getVersion();
    uint64_t getFlashId() { return cachedFlashID.doubleword; }
    void reboot();
    void restartDaemon();
    void ping();

    const std::shared_ptr<RegMappedClient> client;

private:
    union flash_id cachedFlashID;
};

};  // namespace Canmore
