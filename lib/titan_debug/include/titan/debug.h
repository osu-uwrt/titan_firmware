#ifndef TITAN__DEBUG_H_
#define TITAN__DEBUG_H_

#include "canmore/reg_mapped/server.h"

#include <stdint.h>
#include <stdio.h>

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
 * @note This library requires titan_safety to function properly. Without titan_safety, this library has no way to
 * monitor chip status or reboot into bootloader mode.
 *
 */

/**
 * @brief Initialize new CANmore Debug Interface
 *
 * @param tx_func Transmit function to use during CAN transmissions
 * @param multiword_buffer_len The length in bytes of the multiword response buffer to allocate (0 to disable multiword)
 */
void debug_init(reg_mapped_server_tx_func tx_func, size_t multiword_buffer_len);

/**
 * @brief Process a new message for the CANmore debug interface
 *
 * @param msg_buffer Buffer containing message
 * @param len Length of message
 */
void debug_process_message(uint8_t *msg_buffer, size_t len);

#endif

/**
 * @brief Callback typedef for canmore debug remote command handlers.
 *
 * This follows the standard POSIX format of argc and argv, as well as returning a status code.
 * Additionally, a file is provided for command output. Any data written to this file will be sent back over CAN bus.
 *
 * @param argc Number of arguments, including the command name (argument 0)
 * @param argv Array of string arguments. Index 0 is the command name. Terminated with a NULL value
 * @param fout A file which can be written to using stdio functions (fprintf, fputs, etc.). This can be used like stdio
 *
 * @return Integer status code. Zero status code is normal, non-zero status codes are sent back as a failure.
 *         These status codes can be used for whatever purpose.
 */
typedef int (*debug_remote_cmd_cb)(size_t argc, const char *const *argv, FILE *fout);

/**
 * @brief Registers a new custom remote command for Canmore CLI. This allows easy development as it exposes a similar
 * interface to standard POSIX applications.
 *
 * The callback is executed as part of the canbus_tick in the main application, when processing utility frames. It is
 * safe to perform almost any application in part of this callback.
 *
 * @param name The command name to register. This name must not conflict with any other commands (else it will panic)
 * @param usage The argument usage for the command. (Example: "[a] [b]" for a command add which accepts two arguments)
 * @param help_msg The help message to display for this command (Example: "Adds the two arguments [a] and [b] together")
 * @param callback The function callback which will be called when this command is executed by Canmore CLI
 */
void debug_remote_cmd_register(const char *name, const char *usage, const char *help_msg, debug_remote_cmd_cb callback);

#endif
