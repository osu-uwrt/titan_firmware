#include "canmore_cpp/RegMappedClient.hpp"

#include "titan/canmore.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

using namespace Canmore;

RegMappedEthernetClient::RegMappedEthernetClient(struct in_addr ipAddr, uint16_t port): socketFd(-1) {
    // Store the parameters
    destaddr.sin_family = AF_INET;
    destaddr.sin_addr = ipAddr;
    destaddr.sin_port = htons(port);

    // Open socket
    if ((socketFd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
    }

    // Bind to receive UDP packets
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr = { 0 };
    addr.sin_port = 0;

    if (bind(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        throw std::system_error(errno, std::generic_category(), "bind");
    }

    // Configure reg_mapped_client struct for linuxmappings
    clientCfg.tx_func = &clientTxCB;
    clientCfg.clear_rx_func = &clearRxCB;
    clientCfg.rx_func = &clientRxCB;
    clientCfg.transfer_mode = TRANSFER_MODE_SINGLE;  // TODO: Switch to multiword when implemented
    clientCfg.arg = this;
    clientCfg.timeout_ms = REG_MAPPED_TIMEOUT_MS;
    clientCfg.max_in_flight = REG_MAPPED_MAX_IN_FLIGHT_PACKETS_ETH;
}

RegMappedEthernetClient::~RegMappedEthernetClient() {
    if (socketFd >= 0) {
        close(socketFd);
    }
}

void RegMappedEthernetClient::sendRaw(const std::vector<uint8_t> data) {
    if (!clientTx(data.data(), data.size())) {
        throw RegMappedClientError(REG_MAPPED_CLIENT_RESULT_TX_FAIL, data.size());
    }
}

bool RegMappedEthernetClient::clientTx(const uint8_t *buf, size_t len) {
    if (sendto(socketFd, buf, len, 0, (sockaddr *) &destaddr, sizeof(destaddr)) != (ssize_t) len) {
        return false;
    }

    return true;
}

bool RegMappedEthernetClient::clientRx(uint8_t *buf, size_t len, unsigned int timeoutMs) {
    struct pollfd fds[1] = {};
    fds[0].fd = socketFd;
    fds[0].events = POLLIN;

    if (poll(fds, 1, timeoutMs) <= 0) {
        // Either timeout or error
        return false;
    }

    if (!(fds[0].revents & POLLIN)) {
        // Poll didn't return a POLLIN event, so that means that we must have gotten an error
        return false;
    }

    struct sockaddr_in recvaddr;
    socklen_t recvaddrlen = sizeof(recvaddr);
    if (recvfrom(socketFd, buf, len, 0, (sockaddr *) &recvaddr, &recvaddrlen) != (ssize_t) len) {
        // Error reading from socket
        return false;
    }

    if (recvaddr.sin_family != destaddr.sin_family || recvaddr.sin_addr.s_addr != destaddr.sin_addr.s_addr ||
        recvaddr.sin_port != destaddr.sin_port) {
        // Unexpected packet source
        return false;
    }

    return true;
}

bool RegMappedEthernetClient::clearRx(void) {
    uint8_t emptybuf;
    while (recv(socketFd, &emptybuf, sizeof(emptybuf), 0) > 0) {
    }

    return true;
}
