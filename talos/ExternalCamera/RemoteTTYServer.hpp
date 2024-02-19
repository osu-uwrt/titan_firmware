#pragma once

#include "canmore_cpp/CANSocket.hpp"
#include "canmore_cpp/RemoteTTYStream.hpp"

#include "titan/canmore.h"

namespace Canmore {

class RemoteTTYServerEventHandler {
    friend class RemoteTTYServer;

protected:
    virtual void handleStdin(const std::span<const uint8_t> &data) = 0;
    virtual void handleWindowSize(uint16_t rows, uint16_t cols) = 0;
    virtual void handleStdioReady() = 0;
};

class RemoteTTYServer : public CANSocket, public RemoteTTYStreamRXCallback, public RemoteTTYStreamTXCallback {
public:
    RemoteTTYServer(RemoteTTYServerEventHandler &handler, int ifIndex, uint8_t clientId);
    ~RemoteTTYServer();

    void disconnect(bool isError);

    void stdioNotifyWhenReady();
    bool stdioCanWrite();
    void stdoutWrite(const std::span<const uint8_t> &data);
    void stderrWrite(const std::span<const uint8_t> &data);

    bool isDisconnected() { return disconnected_; }

    bool isDisconnected(bool &inError) {
        if (disconnected_)
            inError = disconnectedInError_;
        else
            inError = false;
        return disconnected_;
    }

    // Disabling copying of RemoteTTYClient (since we have a file discriptor)
    RemoteTTYServer(RemoteTTYServer const &) = delete;
    RemoteTTYServer &operator=(RemoteTTYServer const &) = delete;

    const uint8_t clientId;

protected:
    // Handle callback interfaces
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;
    void transmitStreamPacket(uint8_t streamId, uint16_t seqNum, const std::span<const uint8_t> &data) override;
    void notifyBufferAvailable() override;
    void transmitAck(uint16_t seqNum) override;

    // Need to override PollFD to enable timer support
    void handleEvent(const pollfd &fd) override;

public:
    void populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) override;

private:
    bool disconnected_ = false;
    bool disconnectedInError_ = false;
    bool notifyRequested_ = false;
    int timerFd_ = -1;
    std::shared_ptr<PollFDDescriptor> timerPollDescriptor_;
    RemoteTTYServerEventHandler &handler_;
    RemoteTTYStreamTXScheduler txScheduler_;
    RemoteTTYStreamRXScheduler rxScheduler_;

    void sendControlCommand(uint16_t cmd, const std::span<const uint8_t> &data);
    void handleControlCommand(uint16_t cmd, const std::span<const uint8_t> &data);
    void tickRxScheduler();
};

};  // namespace Canmore
