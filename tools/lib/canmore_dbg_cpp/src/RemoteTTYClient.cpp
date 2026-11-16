#include "canmore_cpp/RemoteTTYClient.hpp"

#include "canmore_cpp/FdCompat.hpp"

#include <system_error>
#include <unistd.h>

using namespace Canmore;

RemoteTTYClient::RemoteTTYClient(RemoteTTYClientEventHandler &handler_, const std::string &canInterface,
                                 uint8_t clientId):
    CANTransport(canInterface), clientId(clientId), handler_(handler_), txScheduler_(*this), rxScheduler_(*this) {
    // Create the timer fd so we can properly time the receiver callback
    timerFd_ = FdCompat::createTimer();

    timerPollDescriptor_ = PollFDDescriptor::create(*this, timerFd_, POLLIN);

    // Configure the CANTransport to receive all packets for the remote tty interface
    // The remote tty interface only uses extended frame packets, so we just watch for those
    struct can_filter rfilter[] = { { .can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, 0, 0),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_REMOTE_TTY_ID_MASK) } };
    setRxFilters(std::span<can_filter> { rfilter });

    // Start the timerfd
    // rxScheduler_ also requires that we call receiverHandleTimer() once at startup to begin transmission
    tickRxScheduler();
}

RemoteTTYClient::~RemoteTTYClient() {
    if (timerFd_ >= 0) {
        close(timerFd_);
        timerFd_ = -1;
    }

    // Disconnect, reporting eror since the owner didn't clean us up properly
    disconnect(true);
}

void RemoteTTYClient::disconnect(bool isError) {
    // Don't disconnect twice
    if (disconnected_) {
        return;
    }

    canmore_remote_tty_cmd_disconnect_t pkt = { .pkt = { .is_err = isError } };
    sendControlCommand(CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID, { pkt.data, sizeof(pkt) });

    disconnected_ = true;
    disconnectedInError_ = isError;
}
void RemoteTTYClient::sendWindowSize(uint16_t rows, uint16_t cols) {
    canmore_remote_tty_cmd_window_size_t pkt = { .pkt = { .rows = rows, .cols = cols } };
    sendControlCommand(CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_ID, { pkt.data, sizeof(pkt) });
}

void RemoteTTYClient::stdinNotifyWhenReady() {
    notifyRequested_ = true;
}

bool RemoteTTYClient::stdinCanWrite() {
    return txScheduler_.spaceAvailable();
}

void RemoteTTYClient::stdinWrite(const std::span<const uint8_t> &data) {
    if (!disconnected_) {
        txScheduler_.write(CANMORE_REMOTE_TTY_SUBCH_STDIN, data);
    }
}

void RemoteTTYClient::handleFrame(const CANFrame &frame) {
    canid_t can_id = frame.id;
    auto data = frame.payload();

    // Extract subch and seqCmd
    uint8_t subch = (can_id >> CANMORE_REMOTE_TTY_ID_SUBCH_OFFSET) & ((1 << CANMORE_REMOTE_TTY_ID_SUBCH_LENGTH) - 1);
    uint16_t seqCmd =
        (can_id >> CANMORE_REMOTE_TTY_ID_SEQ_CMD_OFFSET) & ((1 << CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH) - 1);

    // Sanity check that the filters worked
    if (can_id != (CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, subch, seqCmd))) {
        throw std::logic_error("Remote TTY CAN filters broke??");
    }

    // Call the appropriate handlers
    if (subch == CANMORE_REMOTE_TTY_SUBCH_CONTROL) {
        handleControlCommand(seqCmd, data);
    }
    else {
        // Not control, must be stream. Try to handle it
        // If the check fails, then just drop the packet
        if (rxScheduler_.checkPacket(seqCmd)) {
            auto result1 = std::find_if(std::rbegin(data), std::rend(data), [](auto &v) { return v != 0; });
            if (result1 != std::rend(data)) {
                size_t trimmedLength = std::distance(std::begin(data), (result1 + 1).base()) + 1;
                if (subch == CANMORE_REMOTE_TTY_SUBCH_STDERR) {
                    handler_.handleStderr(std::span<const uint8_t>(data.data(), trimmedLength));
                }
                else if (subch == CANMORE_REMOTE_TTY_SUBCH_STDOUT) {
                    handler_.handleStdout(std::span<const uint8_t>(data.data(), trimmedLength));
                }
            }
        }
    }
}

void RemoteTTYClient::transmitStreamPacket(uint8_t streamId, uint16_t seqNum, const std::span<const uint8_t> &data) {
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_A2C(clientId, streamId, seqNum);
    transmitFrame(can_id, data);
}

void RemoteTTYClient::notifyBufferAvailable() {
    if (notifyRequested_) {
        notifyRequested_ = false;
        handler_.handleStdinReady();
    }
}

void RemoteTTYClient::transmitAck(uint16_t seqNum) {
    canmore_remote_tty_cmd_ack_t pkt = { .pkt = { .last_seq_num = seqNum } };
    sendControlCommand(CANMORE_REMOTE_TTY_CMD_ACK_ID, { pkt.data, sizeof(pkt) });
}

void RemoteTTYClient::populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) {
    // Register the CANTransport fds
    CANTransport::populateFds(descriptors);

    // Also register our timer fd
    descriptors.push_back(timerPollDescriptor_);
}

void RemoteTTYClient::handleEvent(const pollfd &fd) {
    // If it's our timer fd, handle it
    if (fd.fd == timerFd_) {
        // Clear the timer fd, we don't care about the data since it's only a single shot timer
        FdCompat::clearEvent(timerFd_);

        // Only tick if we're not disconnected_
        if (!disconnected_) {
            // Tick the rx scheduler (and also reschedule the timer)
            tickRxScheduler();
        }
    }
    // If not, forward it up to the CANTransport
    else {
        CANTransport::handleEvent(fd);
    }
}

void RemoteTTYClient::sendControlCommand(uint16_t cmd, const std::span<const uint8_t> &data) {
    if (disconnected_)
        return;

    // Compute the ID for this command, then send it
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_A2C(clientId, CANMORE_REMOTE_TTY_SUBCH_CONTROL, cmd);
    transmitFrame(can_id, data);
}

void RemoteTTYClient::handleControlCommand(uint16_t cmd, const std::span<const uint8_t> &data) {
    if (cmd == CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID && data.size_bytes() == CANMORE_REMOTE_TTY_CMD_DISCONNECT_LEN) {
        canmore_remote_tty_cmd_disconnect_t disconnectPkt;
        std::copy(data.begin(), data.end(), disconnectPkt.data);
        disconnected_ = true;
        disconnectedInError_ = disconnectPkt.pkt.is_err;
    }
    else if (cmd == CANMORE_REMOTE_TTY_CMD_ACK_ID && data.size_bytes() == CANMORE_REMOTE_TTY_CMD_ACK_LEN) {
        canmore_remote_tty_cmd_ack_t ack;
        std::copy(data.begin(), data.end(), ack.data);
        txScheduler_.notifyAck(ack.pkt.last_seq_num);
    }
}

void RemoteTTYClient::tickRxScheduler() {
    // Schedule the timer to fire for the time requested by the rxScheduler_
    long nextTimerFireMs = (long) rxScheduler_.handleTimer();
    FdCompat::setTimer(timerFd_, nextTimerFireMs);
}
