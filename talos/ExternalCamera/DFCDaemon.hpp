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

    bool stopRequested() { return shouldStop; }

private:
    bool shouldStop = false;

    bool restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
};

class PTYMaster {
public:
    PTYMaster();
};

class CanmoreTTYServer : public Canmore::RemoteTTYServerEventHandler, public Canmore::PollFD {
public:
    CanmoreTTYServer(int ifIndex, uint8_t clientId);

    void populateFds(std::vector<std::pair<Canmore::PollFDHandler *, pollfd>> &fds) override;

protected:
    void handleStdin(const std::span<const uint8_t> &data) override;
    void handleWindowSize(uint16_t rows, uint16_t cols) override;
    void handleStdioReady() override;

private:
    Canmore::RemoteTTYServer server_;
};
