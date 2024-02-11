#pragma once

#include "canmore_cpp/CANSocket.hpp"
#include "canmore_cpp/RemoteTTYStream.hpp"

#include "titan/canmore.h"

namespace Canmore {

class RemoteTTYClientEventHandler {
    friend class RemoteTTYClient;

protected:
    virtual void handleStdout(const std::span<const uint8_t> &data) = 0;
    virtual void handleStderr(const std::span<const uint8_t> &data) = 0;
    virtual void handleStdinReady() = 0;
};

class RemoteTTYClient : public CANSocket, public RemoteTTYStreamRXCallback, public RemoteTTYStreamTXCallback {
public:
    RemoteTTYClient(RemoteTTYClientEventHandler &handler, int ifIndex, uint8_t clientId);
    ~RemoteTTYClient();

    void disconnect(bool isError);
    void sendWindowSize(uint16_t rows, uint16_t cols);

    void stdinNotifyWhenReady();
    bool stdinCanWrite();
    void stdinWrite(const std::span<const uint8_t> &data);

    bool isDisconnected(bool &inError) {
        if (disconnected_)
            inError = disconnectedInError_;
        else
            inError = false;
        return disconnected_;
    }

    // Disabling copying of RemoteTTYClient (since we have a file discriptor)
    RemoteTTYClient(RemoteTTYClient const &) = delete;
    RemoteTTYClient &operator=(RemoteTTYClient const &) = delete;

    const uint8_t clientId;

protected:
    // Handle callback interfaces
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;
    void transmitStreamPacket(uint8_t streamId, uint16_t seqNum, const std::span<const uint8_t> &data) override;
    void notifyBufferAvailable() override;
    void transmitAck(uint16_t seqNum) override;

    // Need to override PollFD to enable timer support
    void populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) override;
    void handleEvent(const pollfd &fd) override;

private:
    bool disconnected_ = false;
    bool disconnectedInError_ = false;
    bool notifyRequested_ = false;
    int timerFd_ = -1;
    RemoteTTYClientEventHandler &handler_;
    RemoteTTYStreamTXScheduler txScheduler_;
    RemoteTTYStreamRXScheduler rxScheduler_;

    void sendControlCommand(uint16_t cmd, const std::span<const uint8_t> &data);
    void handleControlCommand(uint16_t cmd, const std::span<const uint8_t> &data);
    void tickRxScheduler();
};

};  // namespace Canmore
