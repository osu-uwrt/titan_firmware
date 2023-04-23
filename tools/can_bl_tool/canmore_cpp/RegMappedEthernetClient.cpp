#include <algorithm>
#include <system_error>
#include <cstring>

#include <poll.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/socket.h>

#include "canmore/protocol.h"
#include "canmore_titan/reg_mapped_client.h"
#include "canmore_titan/ethernet_defs.h"
#include "RegMappedClient.hpp"

using namespace Canmore;

// The W25Q16JV datasheet specifies max sector erase time to be 400ms
#define REG_MAPPED_TIMEOUT_MS 500

RegMappedEthernetClient::RegMappedEthernetClient(struct in_addr ipAddr, uint16_t port) :
    socketFd(-1)
{
    // Store the parameters
    destaddr.sin_family = AF_INET;
    destaddr.sin_addr = ipAddr;
    destaddr.sin_port = htons(port);

    // Open socket
	if ((socketFd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
	}

    // Bind to receive UDP packets
    struct sockaddr_in addr = {};
	addr.sin_family = AF_INET;
    addr.sin_addr = {0};
    addr.sin_port = 0;

	if (bind(socketFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        throw std::system_error(errno, std::generic_category(), "bind");
	}

    // Configure reg_mapped_client struct for linuxmappings
    clientCfg.tx_func = &clientTxCB;
    clientCfg.clear_rx_func = &clearRxCB;
    clientCfg.rx_func = &clientRxCB;
    clientCfg.transfer_mode = TRANSFER_MODE_SINGLE; // TODO: Switch to multiword when implemented
    clientCfg.arg = this;
    clientCfg.timeout_ms = REG_MAPPED_TIMEOUT_MS;
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

bool RegMappedEthernetClient::clientTx(const uint8_t* buf, size_t len) {
    if (sendto(socketFd, buf, len, 0, (sockaddr*)&destaddr, sizeof(destaddr)) != (ssize_t) len) {
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

    struct sockaddr_in recvaddr;
    socklen_t recvaddrlen = sizeof(recvaddr);
 	if (recvfrom(socketFd, buf, len, 0, (sockaddr*)&recvaddr, &recvaddrlen) != (ssize_t)len) {
        // Error reading from socket
		return false;
	}

    if (recvaddr.sin_family != destaddr.sin_family || recvaddr.sin_addr.s_addr != destaddr.sin_addr.s_addr
         || recvaddr.sin_port != destaddr.sin_port) {
        // Unexpected packet source
        return false;
    }

    return true;
}

bool RegMappedEthernetClient::clearRx(void) {
    uint8_t emptybuf;
    while (recv(socketFd, &emptybuf, sizeof(emptybuf), 0) > 0) {}

    return true;
}
