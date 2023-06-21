#ifndef TITAN__DEBUG_H_
#define TITAN__DEBUG_H_

#include <stdint.h>
#include "titan/canmore.h"

#if TITAN_SAFETY

/**
 * @file titan/debug.h
 *
 * @brief Library to allow for remote debugging over the CANmore reg mapped protocol.
 *
 * This does not have to be sent over CAN bus, but can occur over any protocol that permits packet-based communication
 * with an MTU of at least 8 bytes. Packets can be fed into the process message, and tx_func will be called with the
 * response.
 *
 * @note This library requires titan_safety to function properly. Without titan_safety, this library has no way to monitor
 * chip status or reboot into bootloader mode.
 *
 */

/**
 * @brief Initialize new CANmore Debug Interface
 *
 * @param tx_func Transmit function to use during CAN transmissions
 */
void debug_init(reg_mapped_server_tx_func tx_func);

/**
 * @brief Process a new message for the CANmore debug interface
 *
 * @param msg_buffer Buffer containing message
 * @param len Length of message
 */
void debug_process_message(uint8_t *msg_buffer, size_t len);

#endif

#endif