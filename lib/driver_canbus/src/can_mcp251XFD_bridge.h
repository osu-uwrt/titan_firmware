#ifndef CAN_MCP251XFD_BRIDGE_H
#define CAN_MCP251XFD_BRIDGE_H

#include "canmore/msg_encoding.h"
#include "driver/canbus.h"

#include <stdbool.h>

// PICO_CONFIG: CAN_MCP251XFD_HEARTBEAT_TIMEOUT_MS, Time in milliseconds since last successful heartbeat transmission when the CAN bus is considered offline, type=int, min=1, default=2000, group=driver_canbus
#ifndef CAN_MCP251XFD_HEARTBEAT_TIMEOUT_MS
#define CAN_MCP251XFD_HEARTBEAT_TIMEOUT_MS 2000
#endif

// Note this is necessary due to the strange behavior of the chip randomly locking out transmissions in edge cases.
// Rather than trying to solve all of these issues, it'll just reinitialize every so often if we haven't transmitted a
// heartbeat in a while
// PICO_CONFIG: CAN_MCP251XFD_OFFLINE_RESET_TIMEOUT_MS, Time in milliseconds since CAN bus has gone offline after which the MCP251XFD is reset, type=int, min=1, default=10000, group=driver_canbus
#ifndef CAN_MCP251XFD_OFFLINE_RESET_TIMEOUT_MS
#define CAN_MCP251XFD_OFFLINE_RESET_TIMEOUT_MS 10000
#endif

// ========================================
// Driver Bind Exports
// ========================================

// Message sequence numbers when referring back to messages from the TEF
#define MESSAGE_SEQ_UTILITY_NORMAL 0
#define MESSAGE_SEQ_MSG_FRAME 1
#define MESSAGE_SEQ_UTILITY_HEARTBEAT 2

bool can_mcp251xfd_configure(unsigned int client_id);
void can_mcp251xfd_check_offline_reset(void);

void can_mcp251xfd_report_msg_tx_fifo_ready(void);
void can_mcp251xfd_report_msg_rx_fifo_ready(void);
void can_mcp251xfd_report_utility_tx_fifo_ready(void);
void can_mcp251xfd_report_utility_rx_fifo_ready(void);
void can_mcp251xfd_reset_msg_fifos();

bool can_mcp251x_get_term_state(bool *term_state_out);

/**
 * @brief Disables the GPIO interrupt for the MCP251XFD and saves the previous state for restoring
 *
 * @return uint32_t The previous interrupt state
 */
uint32_t save_and_disable_mcp251Xfd_irq(void);

/**
 * @brief Retores the previous MCP251XFD interrupt state after being diabled
 *
 * @param prev_interrupt_state The previous interrupt state
 */
void restore_mcp251Xfd_irq(uint32_t prev_interrupt_state);

// ========================================
// Interface Exports
// ========================================

bool canbus_msg_driver_space_in_rx(void);
void canbus_msg_driver_post_rx(uint32_t identifier, bool is_extended, size_t data_len, uint8_t *data);

struct utility_message_buffer {
    size_t length;
    uint32_t seq;
    uint8_t channel;
    bool waiting;
    uint8_t data[CANMORE_MAX_FRAME_SIZE];
};

extern struct utility_message_buffer utility_tx_buf;
extern struct utility_message_buffer utility_rx_buf;

/**
 * @brief Holds data waiting to be transmitted
 */
extern canmore_msg_encoder_t msg_encoder;

/**
 * @brief Time after which the last heartbeat transmit has timed out, and the CAN bus is considered offline.
 * This is due to the heartbeat not being ACKed, so if we don't get a successful event for the heartbeat in the
 * TEF we will consider that we have lost the CAN bus.
 */
extern absolute_time_t heartbeat_transmit_timeout;

void canbus_call_internal_error_cb(int line, uint16_t error_code, bool error_code_is_driver_error);
void canbus_call_receive_error_cb(enum canbus_receive_error_codes error_code);

#define canbus_report_driver_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, true)
#define canbus_report_library_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, false)

// Debug Bindings
// TODO: Remove me when can bus bug fixed

void canbus_reenable_intr(void);
void canbus_fifo_clear(void);
void canbus_reset(void);

#endif
