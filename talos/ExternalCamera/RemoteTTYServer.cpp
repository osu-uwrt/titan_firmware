#include "RemoteTTYServer.hpp"

#include <sys/timerfd.h>
#include <system_error>
#include <time.h>
#include <unistd.h>

using namespace Canmore;

RemoteTTYServer::RemoteTTYServer(RemoteTTYServerEventHandler &handler_, int ifIndex, uint8_t clientId):
    CANSocket(ifIndex), clientId(clientId), handler_(handler_), txScheduler_(*this), rxScheduler_(*this) {
    // Create the timer fd so we can properly time the receiver callback
    timerFd_ = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (timerFd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "timerfd_create");
    }

    // Configure the CANSocket to receive all packets for the remote tty interface
    // The remote tty interface only uses extended frame packets, so we just watch for those
    struct can_filter rfilter[] = { { .can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_A2C(clientId, 0, 0),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_REMOTE_TTY_ID_MASK) } };
    setRxFilters(std::span { rfilter });

    // Start the timerfd
    // rxScheduler_ also requires that we call receiverHandleTimer() once at startup to begin transmission
    tickRxScheduler();
}

RemoteTTYServer::~RemoteTTYServer() {
    if (timerFd_ >= 0) {
        close(timerFd_);
        timerFd_ = -1;
    }

    // Disconnect, reporting eror since the owner didn't clean us up properly
    disconnect(true);
}

void RemoteTTYServer::disconnect(bool isError) {
    // Don't disconnect twice
    if (disconnected_) {
        return;
    }

    canmore_remote_tty_cmd_disconnect_t pkt = { .pkt = { .is_err = true } };
    sendControlCommand(CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID, { pkt.data, sizeof(pkt) });

    disconnected_ = true;
    disconnectedInError_ = isError;
}

void RemoteTTYServer::stdioNotifyWhenReady() {
    notifyRequested_ = true;
}

bool RemoteTTYServer::stdioCanWrite() {
    return txScheduler_.transmitterSpaceAvailable();
}

void RemoteTTYServer::stderrWrite(const std::span<const uint8_t> &data) {
    if (!disconnected_) {
        txScheduler_.transmitterWrite(CANMORE_REMOTE_TTY_SUBCH_STDERR, data);
    }
}

void RemoteTTYServer::stdoutWrite(const std::span<const uint8_t> &data) {
    if (!disconnected_) {
        txScheduler_.transmitterWrite(CANMORE_REMOTE_TTY_SUBCH_STDOUT, data);
    }
}

void RemoteTTYServer::handleFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    // Extract subch and seqCmd
    uint8_t subch = (can_id >> CANMORE_REMOTE_TTY_ID_SUBCH_OFFSET) & ((1 << CANMORE_REMOTE_TTY_ID_SUBCH_LENGTH) - 1);
    uint16_t seqCmd =
        (can_id >> CANMORE_REMOTE_TTY_ID_SEQ_CMD_OFFSET) & ((1 << CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH) - 1);

    // Sanity check that the filters worked
    if (can_id != (CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_A2C(clientId, subch, seqCmd))) {
        throw std::logic_error("Remote TTY CAN filters broke?? - " + std::to_string(can_id) + " vs " +
                               std::to_string(CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_A2C(clientId, subch, seqCmd)));
    }

    // Call the appropriate handlers
    if (subch == CANMORE_REMOTE_TTY_SUBCH_CONTROL) {
        handleControlCommand(seqCmd, data);
    }
    else if (subch == CANMORE_REMOTE_TTY_SUBCH_STDIN) {
        // Not control, must be stream. Try to handle it
        // If the check fails, then just drop the packet
        if (rxScheduler_.receiverCheckPacket(seqCmd)) {
            handler_.handleStdin(data);
        }
    }
}

void RemoteTTYServer::transmitStreamPacket(uint8_t streamId, uint16_t seqNum, const std::span<const uint8_t> &data) {
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, streamId, seqNum);
    transmitFrame(can_id, data);
}

void RemoteTTYServer::notifyBufferAvailable() {
    if (notifyRequested_) {
        notifyRequested_ = false;
        handler_.handleStdioReady();
    }
}

void RemoteTTYServer::transmitAck(uint16_t seqNum) {
    canmore_remote_tty_cmd_ack_t pkt = { .pkt = { .last_seq_num = seqNum } };
    sendControlCommand(CANMORE_REMOTE_TTY_CMD_ACK_ID, { pkt.data, sizeof(pkt) });
}

void RemoteTTYServer::populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) {
    // Register the CANSocket fds
    CANSocket::populateFds(fds);

    // Also register our timer fd
    fds.push_back({ this, { .fd = timerFd_, .events = POLLIN, .revents = 0 } });
}

void RemoteTTYServer::handleEvent(const pollfd &fd) {
    // If it's our timer fd, handle it
    if (fd.fd == timerFd_) {
        // Read the timer fd, we don't care about the data since it's only a single shot timer
        uint8_t buf[8];
        read(timerFd_, buf, sizeof(buf));

        // Only tick if we're not disconnected_
        if (!disconnected_) {
            // Tick the rx scheduler (and also reschedule the timer)
            tickRxScheduler();
        }
    }
    // If not, forward it up to the CANSocket
    else {
        CANSocket::handleEvent(fd);
    }
}

void RemoteTTYServer::sendControlCommand(uint16_t cmd, const std::span<const uint8_t> &data) {
    if (disconnected_)
        return;

    // Compute the ID for this command, then send it
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, CANMORE_REMOTE_TTY_SUBCH_CONTROL, cmd);
    transmitFrame(can_id, data);
}

void RemoteTTYServer::handleControlCommand(uint16_t cmd, const std::span<const uint8_t> &data) {
    if (cmd == CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID && data.size_bytes() == CANMORE_REMOTE_TTY_CMD_DISCONNECT_LEN) {
        canmore_remote_tty_cmd_disconnect_t disconnectPkt;
        std::copy(data.begin(), data.end(), disconnectPkt.data);
        disconnected_ = true;
        disconnectedInError_ = disconnectPkt.pkt.is_err;
    }
    else if (cmd == CANMORE_REMOTE_TTY_CMD_ACK_ID && data.size_bytes() == CANMORE_REMOTE_TTY_CMD_ACK_LEN) {
        canmore_remote_tty_cmd_ack_t ack;
        std::copy(data.begin(), data.end(), ack.data);
        txScheduler_.transmitterNotifyAck(ack.pkt.last_seq_num);
    }
    else if (cmd == CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_ID &&
             data.size_bytes() == CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_LEN) {
        canmore_remote_tty_cmd_window_size_t windowSizePkt;
        std::copy(data.begin(), data.end(), windowSizePkt.data);
        handler_.handleWindowSize(windowSizePkt.pkt.rows, windowSizePkt.pkt.cols);
    }
}

void RemoteTTYServer::tickRxScheduler() {
    // Schedule the timer to fire for the time requested by the rxScheduler_
    long nextTimerFireMs = (long) rxScheduler_.receiverHandleTimer();
    itimerspec nextFire = { .it_interval = { .tv_sec = 0, .tv_nsec = 0 },
                            .it_value = { .tv_sec = nextTimerFireMs / 1000,
                                          .tv_nsec = (nextTimerFireMs % 1000) * 1000000 } };

    if (timerfd_settime(timerFd_, 0, &nextFire, NULL) < 0) {
        throw std::system_error(errno, std::generic_category(), "timerfd_settime");
    }
}
