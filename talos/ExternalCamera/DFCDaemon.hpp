#pragma once

#include "RegMappedServer.hpp"

#include "titan/canmore.h"

#define HEARTBEAT_INTERVAL_MS 500

class HeartbeatTransmitter {
public:
    HeartbeatTransmitter(int ifIndex, int clientId);
    ~HeartbeatTransmitter();

    void start();

private:
    int ifIndex;
    int clientId;
    int socketFd;
};

class CanmoreLinuxServer {
public:
    CanmoreLinuxServer(int ifIndex, uint8_t clientId);

    void run();

private:
    Canmore::RegMappedCANServer server;
    bool shouldStop = false;

    bool restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
};
