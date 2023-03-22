#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "canmore/protocol.h"

#include "reg_mapped_client_linux.h"

#define verbose 1

// ========================================
// Callback Functions
// ========================================

static bool reg_mapped_client_linux_tx_cb(const uint8_t *buf, size_t len, void* arg) {
    reg_mapped_client_linux_inst_t *inst = (reg_mapped_client_linux_inst_t *)arg;

    if (len > CAN_MAX_DLEN) {
        return false;
    }

    struct can_frame frame;
    frame.can_id = CANMORE_CALC_UTIL_ID_A2C(inst->client_id, inst->channel);
    frame.can_dlc = len;
    memcpy(frame.data, buf, len);

    if (write(inst->socket, &frame, sizeof(frame)) != sizeof(frame)) {
        if (verbose) perror("CAN write");
		return false;
	}

    return true;
}

static bool reg_mapped_client_linux_clear_rx_cb(void* arg) {
    reg_mapped_client_linux_inst_t *inst = (reg_mapped_client_linux_inst_t *)arg;

    struct can_frame frame;
    while (read(inst->socket, &frame, sizeof(frame)) > 0) {}

    return true;
}

static bool reg_mapped_client_linux_rx_cb(uint8_t *buf, size_t len, unsigned int timeout_ms, void* arg) {
    reg_mapped_client_linux_inst_t *inst = (reg_mapped_client_linux_inst_t *)arg;

    if (len > CAN_MAX_DLEN) {
        return false;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(inst->socket, &rfds);

    struct timeval tv = {timeout_ms/1000, (timeout_ms%1000) * 1000};
    int num_fds;
    if ((num_fds = select(1, &rfds, NULL, NULL, &tv)) < 0) {
        if (verbose) perror("CAN select");
        return false;
    }

    if (!num_fds) {
        // Timeout
        return false;
    }

    struct can_frame frame;
 	if (read(inst->socket, &frame, sizeof(frame)) < 0) {
		perror("CAN read");
		return false;
	}

    if (frame.can_id != CANMORE_CALC_UTIL_ID_C2A(inst->client_id, inst->channel)) {
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

// ========================================
// Configuration Functions
// ========================================

bool reg_mapped_client_linux_open(reg_mapped_client_linux_inst_t *inst, unsigned int if_index, uint8_t client_id, uint8_t channel, uint32_t timeout_ms) {
    inst->if_index = if_index;
    inst->client_id = client_id;
    inst->channel = channel;

    // Open socket
	if ((inst->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        if (verbose) perror("CAN socket");
		return false;
	}

    // Bind to requested interface
    struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = if_index;

	if (bind(inst->socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        if (verbose) perror("CAN bind");
        close(inst->socket);
		return false;
	}

    // Configure filter for specific client/channel
	struct can_filter rfilter[] = {{
        .can_id = CANMORE_CALC_UTIL_ID_C2A(client_id, channel),
        .can_mask = CAN_SFF_MASK
    }};
	if (setsockopt(inst->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        if (verbose) perror("CAN setsockopt");
        close(inst->socket);
        return false;
    }

    // Configure noblocking (for flush)
    int flags = fcntl(inst->socket, F_GETFL, 0);
    if (flags < 0) {
        if (verbose) perror("CAN fnctl F_GETFL");
        close(inst->socket);
        return false;
    }
    if (fcntl(inst->socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        if (verbose) perror("CAN fnctl F_SETFL");
        close(inst->socket);
        return false;
    }

    // Configure reg_mapped_client struct for linuxmappings
    inst->client_cfg.tx_func = reg_mapped_client_linux_tx_cb;
    inst->client_cfg.clear_rx_func = reg_mapped_client_linux_clear_rx_cb;
    inst->client_cfg.rx_func = reg_mapped_client_linux_rx_cb;
    inst->client_cfg.arg = inst;
    inst->client_cfg.timeout_ms = timeout_ms;

    return true;
}

bool reg_mapped_client_linux_close(reg_mapped_client_linux_inst_t *inst) {
    if (close(inst->socket) < 0) {
        if (verbose) perror("CAN close");
        return false;
    }

    return true;
}

// ========================================
// Wrapped Functions
// ========================================

int reg_mapped_client_linux_read_register(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset, uint32_t *data_out) {
    return reg_mapped_client_read_register(&inst->client_cfg, page, offset, data_out);
}

int reg_mapped_client_linux_write_register(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset, uint32_t data) {
    return reg_mapped_client_write_register(&inst->client_cfg, page, offset, data);
}

int reg_mapped_client_linux_read_array(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset_start,
                                        uint32_t *data_array, uint8_t num_words) {
    return reg_mapped_client_read_array(&inst->client_cfg, page, offset_start, data_array, num_words);
}

int reg_mapped_client_linux_write_array(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset_start,
                                        const uint32_t *data_array, uint8_t num_words) {
    return reg_mapped_client_write_array(&inst->client_cfg, page, offset_start, data_array, num_words);
}