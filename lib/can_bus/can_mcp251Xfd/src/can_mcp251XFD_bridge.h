#ifndef CAN_MCP251XFD_BRIDGE_H
#define CAN_MCP251XFD_BRIDGE_H

#include <stdbool.h>

#include "canmore/msg_encoding.h"
#include "can_mcp251Xfd/canbus.h"

// ========================================
// Driver Bind Exports
// ========================================

bool can_mcp251xfd_configure(unsigned int client_id);

void can_mcp251xfd_report_msg_tx_fifo_ready(void);
void can_mcp251xfd_report_msg_rx_fifo_ready(void);
void can_mcp251xfd_report_utility_tx_fifo_ready(void);
void can_mcp251xfd_report_utility_rx_fifo_ready(void);

// ========================================
// Interface Exports
// ========================================

/**
 * @brief Storage for queuing CANmore received message data
*/
struct canmore_received_msg {
    // Message data buffer
    uint8_t data[CANMORE_MAX_MSG_LENGTH];
    // Length of data in buffer
    size_t length;
    // If the message is waiting to be read
    bool waiting;
};

extern struct canmore_received_msg canmore_received_msg;

/**
 * @brief Holds data waiting to be transmitted
*/
extern canmore_msg_encoder_t encoding_buffer;

void canbus_call_internal_error_cb(int line, uint16_t error_code, bool error_code_is_driver_error);
void canbus_call_receive_error_cb(enum canbus_receive_error_codes error_code);

#define canbus_report_driver_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, true)
#define canbus_report_library_error(error_code) canbus_call_internal_error_cb(__LINE__, error_code, false)

#endif