#include "canmore_cpp/CANSocket.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace Canmore;

CANSocket::CANSocket(int ifIndex): ifIndex(ifIndex), socketFd(-1) {
    // Open socket
    if ((socketFd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN socket");
    }

    // Bind to requested interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifIndex;

    if (bind(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(socketFd);
        socketFd = -1;
        throw std::system_error(errno, std::generic_category(), "CAN bind");
    }
}

CANSocket::CANSocket(int ifIndex, const std::span<can_filter> &&rxFilters): CANSocket(ifIndex) {
    // Need to close on failure as destructors aren't called if constructor throws exception
    setRxFiltersInternal(rxFilters, true);
}

CANSocket::~CANSocket() {
    if (socketFd >= 0) {
        close(socketFd);
        socketFd = -1;
    }
}

void CANSocket::setRxFiltersInternal(const std::span<can_filter> &rxFilters, bool closeOnFail) {
    // After performing standard initialization, add the filters
    if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, rxFilters.data(), rxFilters.size_bytes()) < 0) {
        if (closeOnFail) {
            close(socketFd);
            socketFd = -1;
        }
        throw std::system_error(errno, std::generic_category(), "CAN setsockopt");
    }
}

void CANSocket::populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) {
    // Register socketFd for POLLIN events, calling back to this function
    fds.push_back({ this, { .fd = socketFd, .events = POLLIN, .revents = 0 } });
}

void CANSocket::transmitFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    if (data.size() > CAN_MAX_DLEN) {
        throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
    }

    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data.size();
    std::copy_n(data.data(), data.size(), frame.data);

    if (write(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::system_error(errno, std::generic_category(), "CAN write");
    }
}

void CANSocket::clearRxBuffer() {
    struct pollfd fd = { .fd = socketFd, .events = POLLIN, .revents = 0 };

    // Loop until poll reports no data left in rx buffer
    while (true) {
        if (poll(&fd, 1, 0) < 0) {
            throw std::system_error(errno, std::generic_category(), "CAN Flush Poll");
        }
        else if (!(fd.revents & POLLIN)) {
            break;
        }

        struct can_frame frame;
        if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::system_error(errno, std::generic_category(), "CAN Flush Read");
        }
    }
}

void CANSocket::handleEvent(const pollfd &fd) {
    if (fd.revents & POLLIN) {
        struct can_frame frame;
        if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::system_error(errno, std::generic_category(), "CAN read");
        }

        handleFrame(frame.can_id, std::span<const uint8_t>(frame.data, frame.can_dlc));
    }
    if (fd.revents & (POLLERR | POLLHUP)) {
        throw std::runtime_error("CAN socket unexpectedly closed");
    }
}
