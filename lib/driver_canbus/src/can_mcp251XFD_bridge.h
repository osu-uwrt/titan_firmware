#ifndef CAN_MCP251XFD_BRIDGE_H
#define CAN_MCP251XFD_BRIDGE_H

#include <stdbool.h>

#include "driver/canbus.h"
#include "titan/canmore.h"

// ========================================
// Driver Bind Exports
// ========================================

bool can_mcp251xfd_configure(unsigned int client_id);

void can_mcp251xfd_report_msg_tx_fifo_ready(void);
void can_mcp251xfd_report_msg_rx_fifo_ready(void);
void can_mcp251xfd_report_utility_tx_fifo_ready(void);
void can_mcp251xfd_report_utility_rx_fifo_ready(void);
void can_mcp251xfd_reset_msg_fifos();

bool can_mcp251xfd_get_in_error(void);

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
    uint8_t channel;
    size_t length;
    bool waiting;
    uint8_t data[CANMORE_FRAME_SIZE];
};

extern struct utility_message_buffer utility_tx_buf;
extern struct utility_message_buffer utility_rx_buf;

/**
 * @brief Holds data waiting to be transmitted
*/
extern canmore_msg_encoder_t encoding_buffer;

/**
 * @brief True if canbus messages are being processed by the application, false if not
 * Used to avoid buffer overflow errors when messages aren't being processed (as buffer overflows are marked as
 * a system error, which is not the case when we are sent random data after the fact)
 */
extern bool canbus_msg_opened;

void canbus_call_internal_error_cb(int line, uint16_t error_code, bool error_code_is_driver_error);
void canbus_call_receive_error_cb(enum canbus_receive_error_codes error_code);

#define canbus_report_driver_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, true)
#define canbus_report_library_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, false)

#endif