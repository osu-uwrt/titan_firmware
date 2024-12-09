#include "can_mcp251XFD_bridge.h"

#include "canmore/msg_encoding.h"
#include "driver/canbus.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "titan/debug.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

static volatile bool canbus_msg_opened = false;

// ========================================
// Callback Functions
// ========================================

static volatile bool canbus_device_in_error_state = false;

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
        canbus_error_data_t error_data = { .error_code = error_code,
                                           .error_line = line,
                                           .is_driver_error = error_code_is_driver_error };
        canbus_internal_error_cb(error_data);
    }
}

static void report_canmore_msg_decode_error(__unused void *arg, unsigned int error_code) {
    // Only report decode errors if we're listening
    if (canbus_msg_opened) {
        canbus_call_receive_error_cb(CANBUS_RECVERR_DECODE_ERROR_BASE + error_code);
    }
}

void canbus_set_device_in_error(bool device_in_error_state) {
    canbus_device_in_error_state = device_in_error_state;
}

// ========================================
// Msg Queue Functions
// ========================================

canmore_msg_decoder_t msg_decoder = { 0 };
canmore_msg_encoder_t msg_encoder = { 0 };
static uint8_t msg_encoder_buf[CANMORE_MAX_MSG_LENGTH];

struct canmore_received_msg {
    // Message data buffer
    uint8_t data[CANMORE_MAX_MSG_LENGTH];
    // Length of data in buffer
    size_t length;
    // If the message is waiting to be read
    volatile bool waiting;
} canmore_received_msg = { 0 };

// Exports to Application Code

bool canbus_msg_open(void) {
    if (!canbus_initialized) {
        return false;
    }

    // After we set this bool, all write requests should now be allowed and receies will set the waiting flag
    canbus_msg_opened = true;

    return true;
}

void canbus_msg_close(void) {
    assert(canbus_setup);

    canbus_msg_opened = false;
    canmore_received_msg.waiting =
        false;  // Must be cleared after canbus_msg_opened (since it won't mark messages received after this)
}

bool canbus_msg_read_available(void) {
    assert(canbus_setup);

    return canbus_initialized && canbus_msg_opened && canmore_received_msg.waiting;
}

size_t canbus_msg_read(uint8_t *buf, size_t len) {
    assert(canbus_setup);

    if (!canbus_initialized || !canbus_msg_opened) {
        return 0;
    }

    uint32_t prev_interrupt = save_and_disable_mcp251Xfd_irq();

    if (!canmore_received_msg.waiting) {
        restore_mcp251Xfd_irq(prev_interrupt);
        return 0;
    }

    size_t copy_len = canmore_received_msg.length;
    if (copy_len > len) {
        copy_len = len;
    }

    memcpy(buf, canmore_received_msg.data, copy_len);
    canmore_received_msg.waiting = false;

    can_mcp251xfd_report_msg_rx_fifo_ready();

    restore_mcp251Xfd_irq(prev_interrupt);

    return copy_len;
}

bool canbus_msg_write_available(void) {
    assert(canbus_setup);

    return canbus_initialized && canbus_msg_opened && canmore_msg_encode_done(&msg_encoder);
}

size_t canbus_msg_write(const uint8_t *buf, size_t len) {
    assert(canbus_setup);

    if (!canbus_msg_opened || !canbus_initialized) {
        return 0;
    }

    // Ensure that writing to the encode object isn't interrupted by IRQ
    // This is different from all the other values since it's not just a single bool which controls state, but instead
    // the position of two pointers in the object
    uint32_t prev_interrupt = save_and_disable_mcp251Xfd_irq();

    if (!canmore_msg_encode_done(&msg_encoder)) {
        restore_mcp251Xfd_irq(prev_interrupt);
        return 0;
    }

    // First copy the message into the backing buffer
    // Since we service in interrupts, we need this to be copied out from the caller
    if (len > CANMORE_MAX_MSG_LENGTH) {
        len = CANMORE_MAX_MSG_LENGTH;
    }
    memcpy(msg_encoder_buf, buf, len);

    // TODO: Enable message subtype support
    canmore_msg_encode_load(&msg_encoder, 0, msg_encoder_buf, len);
    can_mcp251xfd_report_msg_tx_fifo_ready();

    restore_mcp251Xfd_irq(prev_interrupt);

    return len;
}

// Exports to Driver Code

bool canbus_msg_driver_space_in_rx(void) {
    // No need to check canbus_msg_opened as waiting is cleared after clearing canbus_msg_opened
    // and waiting can never be set with canbus_msg_opened false
    return canbus_initialized && !canmore_received_msg.waiting;
}

void canbus_msg_driver_post_rx(uint32_t identifier, bool is_extended, size_t data_len, uint8_t *data) {
    if (!canbus_msg_driver_space_in_rx()) {
        return;
    }

    size_t msg_size = canmore_msg_decode_frame(&msg_decoder, identifier, is_extended, data, data_len);

    // Only queue the data if the decode was successful and we care to receive messages
    // This check occurs here (and not before decoding) as we don't want to think we got a corrupted message
    // when in reality we just didn't set opened until halfway through receive
    if (msg_size > 0 && canbus_msg_opened) {
        memcpy(canmore_received_msg.data, canmore_msg_decode_get_buf(&msg_decoder), msg_size);
        canmore_received_msg.length = msg_size;
        canmore_received_msg.waiting = true;
    }
}

// ========================================
// Utility Queue Functions
// ========================================

struct utility_message_buffer utility_tx_buf = { 0 };
struct utility_message_buffer utility_rx_buf = { 0 };

bool canbus_utility_frame_read_available(void) {
    assert(canbus_setup);

    return canbus_initialized && utility_rx_buf.waiting;
}

size_t canbus_utility_frame_read(uint32_t *channel_out, uint8_t *buf, size_t len) {
    assert(canbus_setup);

    if (!canbus_initialized) {
        return 0;
    }

    uint32_t prev_interrupt = save_and_disable_mcp251Xfd_irq();

    if (!utility_rx_buf.waiting) {
        restore_mcp251Xfd_irq(prev_interrupt);
        return 0;
    }

    size_t copy_len = utility_rx_buf.length;
    if (copy_len > len)
        copy_len = len;

    memcpy(buf, utility_rx_buf.data, copy_len);
    *channel_out = utility_rx_buf.channel;
    utility_rx_buf.waiting = false;

    can_mcp251xfd_report_utility_rx_fifo_ready();

    restore_mcp251Xfd_irq(prev_interrupt);

    return copy_len;
}

bool canbus_utility_frame_write_available(void) {
    assert(canbus_setup);

    return canbus_initialized && !utility_tx_buf.waiting;
}

size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len) {
    assert(canbus_setup);

    if (!canbus_initialized) {
        return 0;
    }

    // Check channel ID
    if (channel > (1 << CANMORE_NOC_LENGTH)) {
        return 0;
    }

    // Length bounds checking
    if (len > CANMORE_MAX_FRAME_SIZE) {
        len = CANMORE_MAX_FRAME_SIZE;
    }
    if (len == 0) {
        return 0;
    }

    uint32_t prev_interrupt = save_and_disable_mcp251Xfd_irq();

    if (utility_tx_buf.waiting) {
        restore_mcp251Xfd_irq(prev_interrupt);
        return 0;
    }

    // Copy in frame to buffer
    memcpy(utility_tx_buf.data, buf, len);
    utility_tx_buf.length = len;
    utility_tx_buf.channel = channel;
    utility_tx_buf.waiting = true;
    if (channel == CANMORE_CHAN_HEARTBEAT) {
        utility_tx_buf.seq = MESSAGE_SEQ_UTILITY_HEARTBEAT;
    }
    else {
        utility_tx_buf.seq = MESSAGE_SEQ_UTILITY_NORMAL;
    }

    can_mcp251xfd_report_utility_tx_fifo_ready();

    restore_mcp251Xfd_irq(prev_interrupt);

    return len;
}

// ========================================
// Utility Function Callbacks
// ========================================

#define CANBUS_NUM_CHANNELS (1 << CANMORE_NOC_LENGTH)
canbus_utility_chan_cb_t callbacks[CANBUS_NUM_CHANNELS] = { 0 };

void canbus_utility_frame_register_cb(uint32_t channel, canbus_utility_chan_cb_t cb) {
    assert(canbus_setup);
    hard_assert(channel < CANBUS_NUM_CHANNELS && callbacks[channel] == NULL);

    callbacks[channel] = cb;
}

#if TITAN_SAFETY
static int candbg_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
        fprintf(fout, "Usage: candbg [action]\n");
        return 1;
    }

    if (!strcmp(argv[1], "intr_en")) {
        fprintf(fout, "Issuing CAN Interrupt Enable Signal\n");
        canbus_reenable_intr();
        return 0;
    }
    else if (!strcmp(argv[1], "fifo_clear")) {
        fprintf(fout, "Issuing CAN FIFO Reset Signal\n");
        canbus_fifo_clear();
        return 0;
    }
    else if (!strcmp(argv[1], "hw_reset")) {
        fprintf(fout, "Issuing CAN FIFO Reset Signal\n");
        canbus_reset();
        return 0;
    }
    else {
        fprintf(fout, "Invalid Action: '%s'\n", argv[1]);
        return 1;
    }
}

void canbus_control_interface_cb(uint32_t channel, uint8_t *buf, size_t len) {
    if (channel != CANMORE_CHAN_CONTROL_INTERFACE) {
        return;
    }

    debug_process_message(buf, len);
}

void canbus_control_interface_transmit(uint8_t *msg, size_t len, __unused void *arg) {
    canbus_utility_frame_write(CANMORE_CHAN_CONTROL_INTERFACE, msg, len);
}
#endif

// ========================================
// Initialization Functions
// ========================================

bool canbus_setup = false;
bool canbus_initialized = false;
absolute_time_t heartbeat_transmit_timeout = { 0 };

bool canbus_init(unsigned int client_id) {
    // TODO: Enable CAN FD

    // Clear all of the message buffers
    canmore_msg_decode_init(&msg_decoder, report_canmore_msg_decode_error, NULL, false);
    canmore_msg_encode_init(&msg_encoder, client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, false);
    canmore_received_msg.waiting = false;
    utility_tx_buf.waiting = false;
    utility_rx_buf.waiting = false;
    canbus_msg_opened = false;

    if (canbus_setup) {
        // Not first time calling canbus_init, just reenable the chip
        can_mcp251xfd_reenable_chip(client_id);
    }
    else {
        if (!can_mcp251xfd_configure(client_id)) {
            return false;
        }
    }

    bi_decl_if_func_used(bi_program_feature("MCP251XFD CAN Interface"));

    canbus_initialized = true;

    if (!canbus_setup) {
        // These functions only get called once
        canbus_setup = true;

#if TITAN_SAFETY
        // Debug interface only works if safety is compiled in
        // If not, we don't have any way to control the chip's watchdog/query chip status
        debug_init(&canbus_control_interface_transmit, 0);
        canbus_utility_frame_register_cb(CANMORE_CHAN_CONTROL_INTERFACE, &canbus_control_interface_cb);

        debug_remote_cmd_register("candbg", "[action]",
                                  "Issues a debug action to the can bus (trying to solve weird glitches)\n"
                                  "Valid Actions: intr_en, fifo_clear, hw_reset",
                                  candbg_cmd_cb);
#endif
    }

    return true;
}

void canbus_deinit(void) {
    if (!canbus_initialized) {
        return;
    }

    can_mcp251xfd_disable_chip();
    canbus_initialized = false;
}

bool canbus_check_online(void) {
    if (!canbus_initialized) {
        return false;
    }

    return !time_reached(heartbeat_transmit_timeout);
}

absolute_time_t canbus_next_heartbeat = { 0 };
static uint8_t utility_msg_buffer[CANMORE_MAX_FRAME_SIZE];

void canbus_tick(void) {
    if (!canbus_initialized)
        return;

    // Heartbeat scheduling
    if (time_reached(canbus_next_heartbeat) && canbus_utility_frame_write_available()) {
        canbus_next_heartbeat = make_timeout_time_ms(CANMORE_HEARTBEAT_INTERVAL_MS);

        static canmore_heartbeat_t heartbeat = { .data = 0 };

        heartbeat.pkt.cnt += 1;
        heartbeat.pkt.error = canbus_device_in_error_state;
        heartbeat.pkt.mode = CANMORE_CONTROL_INTERFACE_MODE_NORMAL;

        bool term_enabled;
        if (can_mcp251x_get_term_state(&term_enabled)) {
            heartbeat.pkt.term_valid = 1;
            heartbeat.pkt.term_enabled = (term_enabled ? 1 : 0);
        }
        else {
            heartbeat.pkt.term_valid = 0;
            heartbeat.pkt.term_enabled = 0;
        }

        canbus_utility_frame_write(CANMORE_CHAN_HEARTBEAT, &heartbeat.data, sizeof(heartbeat));
    }

    // Handle any utility frames
    if (canbus_utility_frame_read_available()) {
        uint32_t channel;
        size_t len = canbus_utility_frame_read(&channel, utility_msg_buffer, sizeof(utility_msg_buffer));
        if (len && channel < CANBUS_NUM_CHANNELS && callbacks[channel] != NULL) {
            callbacks[channel](channel, utility_msg_buffer, len);
        }
    }

    // Check if we've been offline for too long and need a reset
    can_mcp251xfd_check_offline_reset();
}
