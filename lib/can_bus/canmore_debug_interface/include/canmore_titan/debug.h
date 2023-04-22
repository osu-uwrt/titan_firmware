#ifndef CANMORE_TITAN__DEBUG_H
#define CANMORE_TITAN__DEBUG_H

#include <stdint.h>
#include "canmore_titan/reg_mapped_server.h"

/**
 * @brief Initialize new CANmore Debug Interface
 *
 * @param tx_func Transmit function to use during CAN transmissions
 */
void canmore_debug_init(reg_mapped_server_tx_func tx_func);

/**
 * @brief Process a new message for the CANmore debug interface
 *
 * @param msg_buffer Buffer containing message
 * @param len Length of message
 */
void canmore_debug_process_message(uint8_t *msg_buffer, size_t len);

#endif