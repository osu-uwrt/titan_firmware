#pragma once

#include "RemoteTTYServer.hpp"
#include "canmore_cpp/CANSocket.hpp"
#include "canmore_cpp/RegMappedServer.hpp"

#define HEARTBEAT_INTERVAL_MS 500

class HeartbeatTransmitter {
public:
    HeartbeatTransmitter(int ifIndex, int clientId);
    ~HeartbeatTransmitter();

    void start();

    const int ifIndex;
    const int clientId;

private:
    int socketFd;
};

class CanmoreLinuxServer : public Canmore::RegMappedCANServer {
public:
    CanmoreLinuxServer(int ifIndex, uint8_t clientId);

    bool stopRequested() { return shouldStop_; }
    void getTtyInitialConfig(std::string &termEnv, uint16_t &initialRows, uint16_t &initialCols);
    bool getTtyEnabled() { return remoteTtyEnabled_; }
    void notifyTtyShutdown() { remoteTtyEnabled_ = false; }

private:
    bool shouldStop_ = false;
    bool remoteTtyEnabled_ = false;
    uint32_t windowSzReg_ = 0;
    std::vector<uint8_t> termStrBuf_;

    bool restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
    bool enableTtyCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
};

class PTYMaster {
public:
    PTYMaster();
};

class CanmoreTTYServer : public Canmore::RemoteTTYServerEventHandler, public Canmore::PollFD {
public:
    CanmoreTTYServer(int ifIndex, uint8_t clientId, const std::string &termEnv = "", uint16_t initialRows = 0,
                     uint16_t initialCols = 0);

    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;
    bool isTerminated(bool &inError) { return canmoreServer_.isDisconnected(inError); }

protected:
    void handleStdin(const std::span<const uint8_t> &data) override;
    void handleWindowSize(uint16_t rows, uint16_t cols) override;
    void handleStdioReady() override;

private:
    Canmore::RemoteTTYServer canmoreServer_;
};
