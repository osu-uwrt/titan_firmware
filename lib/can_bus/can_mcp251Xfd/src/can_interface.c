#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "can_mcp251Xfd/canbus.h"

#include "canmore/msg_encoding.h"
#include "canmore/protocol.h"

#include "can_mcp251XFD_bridge.h"


// PICO_CONFIG: CAN_HEARTBEAT_INTERVAL_MS, Interval for CANmore heartbeat transmission over CAN bus in milliseconds, type=int, default=1000, group=can_mcp251Xfd
#ifndef CAN_HEARTBEAT_INTERVAL_MS
#define CAN_HEARTBEAT_INTERVAL_MS 1000
#endif

// TODO: Figure heartbeat timeout stuff
// PICO_CONFIG: CAN_HEARTBEAT_TIMEOUT_MS, Timeout since last successful ACK of heartbeat frame in milliseconds before CAN bus is considered lost. Must be greater than CAN_HEARTBEAT_INTERVAL_MS, type=int, default=1500, group=can_mcp251Xfd
#ifndef CAN_HEARTBEAT_TIMEOUT_MS
#define CAN_HEARTBEAT_TIMEOUT_MS 1500
#endif


// TODO: Figure out if this can be moved with preprocessor when FD support is added to CANMore
// Right now separated to allow setting FD support in this file without needing preprocessor to set in canbus.h
// It's not the best [static allocations don't work with this unfortunately], but it works
const size_t canbus_msg_max_length = CANMORE_MAX_MSG_LENGTH;
const size_t canbus_utility_frame_max_length = CANMORE_FRAME_SIZE;

// ========================================
// Callback Functions
// ========================================

bool canbus_device_in_error_state = false;

// Callback handling
static canbus_receive_error_cb_t canbus_receive_error_cb = NULL;
static canbus_internal_error_cb_t canbus_internal_error_cb = NULL;

void canbus_set_receive_error_cb(canbus_receive_error_cb_t callback) {
    canbus_receive_error_cb = callback;
}

void canbus_set_internal_error_cb(canbus_internal_error_cb_t callback) {
    canbus_internal_error_cb = callback;
}

void canbus_call_receive_error_cb(enum canbus_receive_error_codes error_code) {
    if (canbus_receive_error_cb) {
        canbus_receive_error_cb(error_code);
    }
}

void canbus_call_internal_error_cb(int line, uint16_t error_code, bool error_code_is_driver_error) {
    if (canbus_internal_error_cb) {
        canbus_error_data_t error_data = {
            .error_code = error_code,
            .error_line = line,
            .is_driver_error = error_code_is_driver_error
        };
        canbus_internal_error_cb(error_data);
    }
}

void canbus_set_device_in_error(bool device_in_error_state) {
    canbus_device_in_error_state = device_in_error_state;
}


// ========================================
// Initialization Functions
// ========================================

bool canbus_initialized = false;

bool canbus_init(unsigned int client_id) {
    assert(!canbus_initialized);

    canmore_msg_encode_init(&encoding_buffer, client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT);

    if (!can_mcp251xfd_configure(client_id)) {
        return false;
    }

    canbus_initialized = true;
    return true;
}


bool canbus_check_online(void) {
    assert(canbus_initialized);

    // TODO: Update to new algorithm

    return false; //return !time_reached(canbus_heartbeat_timeout);
}


// ========================================
// Msg Queue Functions
// ========================================

canmore_msg_encoder_t encoding_buffer = {0};
struct canmore_received_msg canmore_received_msg = {0};

bool canbus_msg_read_available(void) {
    assert(canbus_initialized);

    return canmore_received_msg.waiting;
}

size_t canbus_msg_read(uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (!canmore_received_msg.waiting) {
        return 0;
    }

    size_t copy_len = canmore_received_msg.length;
    if (copy_len > len) {
        copy_len = len;
    }

    memcpy(buf, canmore_received_msg.data, copy_len);
    canmore_received_msg.waiting = false;

    can_mcp251xfd_report_msg_rx_fifo_ready();

    return copy_len;
}

bool canbus_msg_write_available(void) {
    assert(canbus_initialized);

    return canmore_msg_encode_done(&encoding_buffer);
}

size_t canbus_msg_write(const uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    // Ensure that write is not interrupted
    uint32_t prev_interrupt = save_and_disable_interrupts();

    if (!canmore_msg_encode_done(&encoding_buffer)) {
        return 0;
    }

    canmore_msg_encode_load(&encoding_buffer, buf, len);

    restore_interrupts(prev_interrupt);

    // TODO: Figure out interrupt and race condition stuff
    can_mcp251xfd_report_msg_tx_fifo_ready();

    return len;
}


// ========================================
// Utility Queue Functions
// ========================================
/*
bool canbus_utility_frame_read_available(void) {
    assert(canbus_initialized);

    return !QUEUE_EMPTY(&utility_receive_queue);
}

size_t canbus_utility_frame_read(uint32_t *channel_out, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_EMPTY(&utility_receive_queue)) {
        return 0;
    }

    // Copy out data from buffer from second core
    struct canmore_utility_frame *frame = QUEUE_CUR_READ_ENTRY(&utility_receive_queue);

    size_t copy_len = frame->dlc;
    if (copy_len > len)
        copy_len = len;

    memcpy(buf, frame->data, copy_len);
    *channel_out = frame->channel;
    QUEUE_MARK_READ_DONE(&utility_receive_queue);

    return copy_len;
}

bool canbus_utility_frame_write_available(void) {
    assert(canbus_initialized);

    return !QUEUE_FULL(&utility_transmit_queue);
}

size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_FULL(&utility_transmit_queue)) {
        return 0;
    }

    // Check channel ID
    if (channel > (1<<CANMORE_NOC_LENGTH)) {
        return 0;
    }

    // Length bounds checking
    if (len > CANMORE_FRAME_SIZE) {
        len = CANMORE_FRAME_SIZE;
    }
    if (len == 0) {
        return 0;
    }

    // Copy in frame to buffer for second core to fetch
    struct canmore_utility_frame *frame = QUEUE_CUR_WRITE_ENTRY(&utility_transmit_queue);
    memcpy(frame->data, buf, len);
    frame->dlc = len;
    frame->channel = channel;
    QUEUE_MARK_WRITE_DONE(&utility_transmit_queue);

    return len;
}
*/