#include <algorithm>
#include <system_error>
#include <cstring>

#include <poll.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "canmore/protocol.h"
#include "canmore_titan/reg_mapped_client.h"
#include "RegMappedClient.hpp"

using namespace Canmore;

// The W25Q16JV datasheet specifies max sector erase time to be 400ms
#define REG_MAPPED_TIMEOUT_MS 500

RegMappedCANClient::RegMappedCANClient(int ifIndex, uint8_t clientId, uint8_t channel) :
    ifIndex(ifIndex), clientId(clientId), channel(channel), socketFd(-1)
{
    // Open socket
	if ((socketFd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)) < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
	}

    // Bind to requested interface
    struct sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifIndex;

	if (bind(socketFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN bind");
	}

    // Configure filter for specific client/channel
	struct can_filter rfilter[] = {{
        .can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, channel),
        .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK)
    }};
	if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN setsockopt");
    }

    // Configure reg_mapped_client struct for linuxmappings
    clientCfg.tx_func = &clientTxCB;
    clientCfg.clear_rx_func = &clearRxCB;
    clientCfg.rx_func = &clientRxCB;
    clientCfg.transfer_mode = TRANSFER_MODE_BULK;
    clientCfg.arg = this;
    clientCfg.timeout_ms = REG_MAPPED_TIMEOUT_MS;
}

RegMappedCANClient::~RegMappedCANClient() {
    if (socketFd >= 0) {
        close(socketFd);
    }
}

void RegMappedCANClient::sendRaw(const std::vector<uint8_t> data) {
    if (!clientTx(data.data(), data.size())) {
        throw RegMappedClientError(REG_MAPPED_CLIENT_RESULT_TX_FAIL);
    }
}

bool RegMappedCANClient::clientTx(const uint8_t* buf, size_t len) {
    if (len > CAN_MAX_DLEN) {
        return false;
    }

    struct can_frame frame;
    frame.can_id = CANMORE_CALC_UTIL_ID_A2C(clientId, channel);
    frame.can_dlc = len;
    memcpy(frame.data, buf, len);

    if (write(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
		return false;
	}

    return true;
}

bool RegMappedCANClient::clientRx(uint8_t *buf, size_t len, unsigned int timeoutMs) {
    if (len > CAN_MAX_DLEN) {
        return false;
    }

    struct pollfd fds[1] = {};
    fds[0].fd = socketFd;
    fds[0].events = POLLIN;

    if (poll(fds, 1, timeoutMs) <= 0) {
        // Either timeout or error
        return false;
    }

    struct can_frame frame;
 	if (read(socketFd, &frame, sizeof(frame)) != sizeof(frame)) {
        // Error reading from socket
		return false;
	}

    if (frame.can_id != CANMORE_CALC_UTIL_ID_C2A(clientId, channel)) {
        // Invalid client ID
        return false;
    }

    if (frame.can_dlc != len) {
        // Unexpected length
        return false;
    }

	memcpy(buf, frame.data, len);

    return true;
}

bool RegMappedCANClient::clearRx(void) {
    struct can_frame frame;
    while (read(socketFd, &frame, sizeof(frame)) > 0) {}

    return true;
}
