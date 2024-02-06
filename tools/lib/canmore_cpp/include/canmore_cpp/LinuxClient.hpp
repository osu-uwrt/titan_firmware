#pragma once

#include "Canmore.hpp"
#include "RegMappedClient.hpp"

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

private:
    union flash_id cachedFlashID;
    std::shared_ptr<RegMappedClient> client;
};

};  // namespace Canmore
