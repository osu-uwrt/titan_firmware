#include "RegMappedServer.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

using namespace Canmore;

RegMappedCANServer::RegMappedCANServer(int ifIndex, uint8_t clientId, uint8_t channel, uint8_t interfaceMode):
    RegMappedServer(interfaceMode), ifIndex(ifIndex), clientId(clientId), channel(channel), socketFd(-1) {
    // Open socket
    if ((socketFd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
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

    // Configure agent to receive agent to client communication on the control interface channel
    struct can_filter rfilter[] = { { .can_id = CANMORE_CALC_UTIL_ID_A2C(clientId, channel),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK) } };
    if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        close(socketFd);
        socketFd = -1;
        throw std::system_error(errno, std::generic_category(), "CAN setsockopt");
    }
}

RegMappedCANServer::~RegMappedCANServer() {
    if (socketFd >= 0) {
        close(socketFd);
    }
}

void RegMappedCANServer::waitForPacket(unsigned int timeoutMs) {
    struct pollfd fds[1] = {};
    fds[0].fd = socketFd;
    fds[0].events = POLLIN;

    // Poll until packet or we reach timeout
    int rc = poll(fds, 1, timeoutMs);
    if (rc == 0) {
        // Timeout, return now
        return;
    }
    else if (rc < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN poll");
    }

    // Actually read the frame
    struct can_frame frame;
    if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::system_error(errno, std::generic_category(), "CAN read");
    }

    if (frame.can_id != CANMORE_CALC_UTIL_ID_A2C(clientId, channel)) {
        throw std::logic_error("Received a packet with invalid CAN ID - Somehow the filters broke?");
    }

    processPacket(std::span<uint8_t>(frame.data, frame.len));
}

void RegMappedCANServer::transmit(const std::span<uint8_t> &data) {
    if (data.size() > CAN_MAX_DLEN) {
        throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
    }

    struct can_frame frame;
    frame.can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, channel);
    frame.can_dlc = data.size();
    memcpy(frame.data, data.data(), data.size());

    if (write(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::system_error(errno, std::generic_category(), "CAN write");
    }
}
