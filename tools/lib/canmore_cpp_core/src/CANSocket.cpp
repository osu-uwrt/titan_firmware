#include "canmore_cpp/CANSocket.hpp"

#include "canmore/msg_encoding.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace Canmore;

static_assert(CANMORE_MAX_FD_FRAME_SIZE == CANFD_MAX_DLEN, "CANmore and Linux definitions do not match");
static_assert(CANMORE_MAX_FRAME_SIZE == CAN_MAX_DLEN, "CANmore and Linux definitions do not match");

// The maximum number of milliseconds to block for when trying to transmit to the socket
// If this elapses before the socket sucessfully writes the data, then that means that the CAN bus has probably broken,
// and no other nodes on the bus are ACKing the frame We don't want to indefinitely stall program execution though, so
// instead we'll error after a reasonable amount of time to be waiting for us to get arbitration onto the bus
// If we are making forward progress, although we'd block under a heavy load, this still shouldn't hit and instead
// rate limit the caller to stay within the max speed of the bus.
#define MAX_BLOCKING_TIME_MS 50

CANSocket::CANSocket(int ifIndex, bool forceNoCanFd): ifIndex(ifIndex), socketFd(-1), useCanFd(false) {
    // Open socket
    if ((socketFd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN socket");
    }

    // Set transmit timeout
    // See comment above constant, but this allows the transmit to rate limit the caller to stay within the bus
    // bandwidth, while also setting a max waiting in the event that the CAN bus completely breaks and stops
    // transmitting
    struct timeval tx_timeout;
    tx_timeout.tv_sec = MAX_BLOCKING_TIME_MS / 1000;
    tx_timeout.tv_usec = MAX_BLOCKING_TIME_MS * 1000;
    if (setsockopt(socketFd, SOL_SOCKET, SO_SNDTIMEO, &tx_timeout, sizeof(tx_timeout))) {
        close(socketFd);
        socketFd = -1;
        throw std::system_error(errno, std::generic_category(), "CAN setsockopt(SO_SNDTIMEO)");
    }

    // Try to enable CAN FD unless it's forced off
    if (!forceNoCanFd) {
        // Create MTU query
        struct ifreq ifr;
        ifr.ifr_ifindex = ifIndex;
        if (!if_indextoname(ifIndex, ifr.ifr_name)) {
            close(socketFd);
            socketFd = -1;
            throw std::system_error(errno, std::generic_category(), "CAN if_indextoname");
        }
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        // Query MTU to see if we have CAN FD support
        if (ioctl(socketFd, SIOCGIFMTU, &ifr) < 0) {
            close(socketFd);
            socketFd = -1;
            throw std::system_error(errno, std::generic_category(), "CAN ioctl(SIOCGIFMTU)");
        }

        // If so, enable it on the socket
        if (ifr.ifr_mtu > 0 && ((size_t) ifr.ifr_mtu) >= CANFD_MTU) {
            useCanFd = true;

            int enableCanFd = 1;
            if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableCanFd, sizeof(enableCanFd))) {
                close(socketFd);
                socketFd = -1;
                throw std::system_error(errno, std::generic_category(),
                                        "CAN setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES)");
            }
        }
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

    // Create poll descriptor for pollfd
    socketPollDescriptor = PollFDDescriptor::create(*this, socketFd, POLLIN);
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

void CANSocket::populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) {
    descriptors.push_back(socketPollDescriptor);
}

bool CANSocket::transmitFrameNoexcept(canid_t can_id, const uint8_t *data, size_t len) noexcept {
    if (!useCanFd) {
        if (len > CAN_MAX_DLEN) {
            return false;
        }

        struct can_frame frame = {};
        frame.can_id = can_id;
        frame.can_dlc = len;
        for (size_t i = 0; i < len; i++) {
            frame.data[i] = data[i];
        }

        return write(socketFd, &frame, sizeof(frame)) == sizeof(frame);
    }
    else {
        if (len > CANFD_MAX_DLEN) {
            return false;
        }
        if (canmore_fd_dlc2len(canmore_fd_len2dlc(len)) != len) {
            return false;
        }

        struct canfd_frame frame = {};
        frame.can_id = can_id;
        frame.flags = CANFD_BRS;
        frame.len = len;
        for (size_t i = 0; i < len; i++) {
            frame.data[i] = data[i];
        }

        return write(socketFd, &frame, sizeof(frame)) != sizeof(frame);
    }
}

void CANSocket::transmitFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    if (!useCanFd) {
        if (data.size() > CAN_MAX_DLEN) {
            throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
        }

        struct can_frame frame = {};
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::copy_n(data.data(), data.size(), frame.data);

        if (write(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::system_error(errno, std::generic_category(), "CAN write");
        }
    }
    else {
        if (data.size() > CANFD_MAX_DLEN) {
            throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
        }
        if (canmore_fd_dlc2len(canmore_fd_len2dlc(data.size())) != data.size()) {
            throw std::logic_error("Invalid CAN FD frame length: " + std::to_string(data.size()));
        }

        struct canfd_frame frame = {};
        frame.can_id = can_id;
        frame.flags = CANFD_BRS;
        frame.len = data.size();
        std::copy_n(data.data(), data.size(), frame.data);

        if (write(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::system_error(errno, std::generic_category(), "CANFD write");
        }
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

        if (!useCanFd) {
            struct can_frame frame;
            if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
                throw std::system_error(errno, std::generic_category(), "CAN Flush Read");
            }
        }
        else {
            struct canfd_frame frame;
            if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
                throw std::system_error(errno, std::generic_category(), "CANFD Flush Read");
            }
        }
    }
}

void CANSocket::handleEvent(const pollfd &fd) {
    if (fd.revents & POLLIN) {
        if (!useCanFd) {
            struct can_frame frame;
            if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
                throw std::system_error(errno, std::generic_category(), "CAN read");
            }

            handleFrame(frame.can_id, std::span<const uint8_t>(frame.data, frame.can_dlc));
        }
        else {
            struct canfd_frame frame;
            if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
                throw std::system_error(errno, std::generic_category(), "CANFD read");
            }

            handleFrame(frame.can_id, std::span<const uint8_t>(frame.data, frame.len));
        }
    }
    if (fd.revents & (POLLERR | POLLHUP)) {
        throw std::runtime_error("CAN socket unexpectedly closed");
    }
}
