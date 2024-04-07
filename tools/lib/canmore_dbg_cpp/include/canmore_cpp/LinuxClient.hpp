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
    void enableRemoteTty(const std::string &term_env, uint16_t windowRows, uint16_t windowCols, const std::string &cmd,
                         bool useUploadWorkingDir);
    void disableRemoteTty();
    bool remoteTtyEnabled();

    void reboot();
    void restartDaemon();
    void ping();

    const std::shared_ptr<RegMappedClient> client;

private:
    union flash_id cachedFlashID;
};

};  // namespace Canmore
