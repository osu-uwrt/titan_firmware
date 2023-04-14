#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "can_mcp251Xfd/canbus.h"

#include "canmore/msg_encoding.h"
#include "canmore_titan/protocol.h"
#include "canmore_titan/debug.h"

#include "can_mcp251XFD_bridge.h"

#include "safety/safety.h"

// PICO_CONFIG: CAN_HEARTBEAT_INTERVAL_MS, Interval for CANmore heartbeat transmission over CAN bus in milliseconds, type=int, default=1000, group=can_mcp251Xfd
#ifndef CAN_HEARTBEAT_INTERVAL_MS
#define CAN_HEARTBEAT_INTERVAL_MS 500
#endif

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

static void report_canmore_msg_decode_error(__unused void* arg, unsigned int error_code) {
    canbus_call_receive_error_cb(CANBUS_RECVERR_DECODE_ERROR_BASE + error_code);
}

void canbus_set_device_in_error(bool device_in_error_state) {
    canbus_device_in_error_state = device_in_error_state;
}


// ========================================
// Msg Queue Functions
// ========================================

canmore_msg_decoder_t msg_decoder = {0};
canmore_msg_encoder_t encoding_buffer = {0};
struct canmore_received_msg {
    // Message data buffer
    uint8_t data[CANMORE_MAX_MSG_LENGTH];
    // Length of data in buffer
    size_t length;
    // If the message is waiting to be read
    bool waiting;
} canmore_received_msg = {0};

// Exports to Application Code

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

    // Ensure that writing to the encode object isn't interrupted by IRQ
    // This is different from all the other values since it's not just a single bool which controls state, but instead
    // the position of two pointers in the object
    uint32_t prev_interrupt = save_and_disable_mcp251Xfd_irq();

    if (!canmore_msg_encode_done(&encoding_buffer)) {
        restore_mcp251Xfd_irq(prev_interrupt);
        return 0;
    }

    canmore_msg_encode_load(&encoding_buffer, buf, len);
    can_mcp251xfd_report_msg_tx_fifo_ready();

    restore_mcp251Xfd_irq(prev_interrupt);

    return len;
}

// Exports to Driver Code

bool canbus_msg_driver_space_in_rx(void) {
    return !canmore_received_msg.waiting;
}

void canbus_msg_driver_post_rx(uint32_t identifier, bool is_extended, size_t data_len, uint8_t *data) {
    if (!canbus_msg_driver_space_in_rx()) {
        return;
    }

    canmore_id_t client_id = {.identifier = identifier};

    if (!is_extended) {
        canmore_msg_decode_frame(&msg_decoder, client_id.pkt_std.noc, data, data_len);
    } else {
        size_t msg_size = canmore_msg_decode_last_frame(&msg_decoder, client_id.pkt_ext.noc,
                                                        data, data_len, client_id.pkt_ext.crc,
                                                        canmore_received_msg.data);

        // Only queue the data if the decode was successful
        if (msg_size > 0) {
            canmore_received_msg.length = msg_size;
            canmore_received_msg.waiting = true;
        }
    }
}

// ========================================
// Utility Queue Functions
// ========================================

struct utility_message_buffer utility_tx_buf = {0};
struct utility_message_buffer utility_rx_buf = {0};

bool canbus_utility_frame_read_available(void) {
    assert(canbus_initialized);

    return utility_rx_buf.waiting;
}

size_t canbus_utility_frame_read(uint32_t *channel_out, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (!utility_rx_buf.waiting) {
        return 0;
    }

    size_t copy_len = utility_rx_buf.length;
    if (copy_len > len)
        copy_len = len;

    memcpy(buf, utility_rx_buf.data, copy_len);
    *channel_out = utility_rx_buf.channel;
    utility_rx_buf.waiting = false;

    can_mcp251xfd_report_utility_rx_fifo_ready();

    return copy_len;
}

bool canbus_utility_frame_write_available(void) {
    assert(canbus_initialized);

    return !utility_tx_buf.waiting;
}

size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (utility_tx_buf.waiting) {
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

    // Copy in frame to buffer
    memcpy(utility_tx_buf.data, buf, len);
    utility_tx_buf.length = len;
    utility_tx_buf.channel = channel;
    utility_tx_buf.waiting = true;

    can_mcp251xfd_report_utility_tx_fifo_ready();

    return len;
}

// ========================================
// Utility Function Callbacks
// ========================================

#define CANBUS_NUM_CHANNELS (1<<CANMORE_NOC_LENGTH)
canbus_utility_chan_cb_t callbacks[CANBUS_NUM_CHANNELS] = {0};

void canbus_utility_frame_register_cb(uint32_t channel, canbus_utility_chan_cb_t cb) {
    assert(canbus_initialized);
    hard_assert(channel < CANBUS_NUM_CHANNELS && callbacks[channel] == NULL);

    callbacks[channel] = cb;
}

void canbus_control_interface_cb(uint32_t channel, uint8_t *buf, size_t len) {
    if (channel != CANMORE_TITAN_CHAN_CONTROL_INTERFACE) {
        return;
    }

    canmore_debug_process_message(buf, len);
}

void canbus_control_interface_transmit(uint8_t *msg, size_t len) {
    canbus_utility_frame_write(CANMORE_TITAN_CHAN_CONTROL_INTERFACE, msg, len);
}


// ========================================
// Initialization Functions
// ========================================

bool canbus_initialized = false;

bool canbus_init(unsigned int client_id) {
    assert(!canbus_initialized);

    canmore_msg_decode_init(&msg_decoder, report_canmore_msg_decode_error, NULL);
    canmore_msg_encode_init(&encoding_buffer, client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT);

    if (!can_mcp251xfd_configure(client_id)) {
        return false;
    }

    canmore_debug_init(&canbus_control_interface_transmit);
    canbus_utility_frame_register_cb(CANMORE_TITAN_CHAN_CONTROL_INTERFACE, &canbus_control_interface_cb);

    canbus_initialized = true;
    return true;
}


bool canbus_check_online(void) {
    assert(canbus_initialized);

    return !can_mcp251xfd_get_in_error();
}

absolute_time_t canbus_next_heartbeat = {0};
static uint8_t msg_buffer[CANMORE_FRAME_SIZE];

void canbus_tick(void) {
    assert(canbus_initialized);

    // Heartbeat scheduling
    if (time_reached(canbus_next_heartbeat) && canbus_utility_frame_write_available()) {
        canbus_next_heartbeat = make_timeout_time_ms(CAN_HEARTBEAT_INTERVAL_MS);

        static canmore_titan_heartbeat_t heartbeat = {.data = 0};

        heartbeat.pkt.cnt += 1;
        heartbeat.pkt.error = (*fault_list_reg) != 0;
        heartbeat.pkt.mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL;

        bool term_enabled;
        if (can_mcp251x_get_term_state(&term_enabled)) {
            heartbeat.pkt.term_valid = 1;
            heartbeat.pkt.term_enabled = (term_enabled ? 1 : 0);
        } else {
            heartbeat.pkt.term_valid = 0;
            heartbeat.pkt.term_enabled = 0;
        }

        canbus_utility_frame_write(CANMORE_CHAN_HEARTBEAT, &heartbeat.data, sizeof(heartbeat));
    }

    // Handle any utility frames
    if (canbus_utility_frame_read_available()) {
        uint32_t channel;
        size_t len = canbus_utility_frame_read(&channel, msg_buffer, sizeof(msg_buffer));
        if (channel < CANBUS_NUM_CHANNELS && callbacks[channel] != NULL) {
            callbacks[channel](channel, msg_buffer, len);
        }
    }
}
