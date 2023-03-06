#ifndef CAN_BL_INTERFACE_H
#define CAN_BL_INTERFACE_H

/**
 * @file can_bl_interface.h
 * @brief Stripped down CAN bus interface for bootloader communciation
 *
 * Many features are removed from this interface, keeping only the essentials to establish communication.
 * Fault handling is also not considered in this application. Errors are only processed during init, where the
 * bootloader can exit without attempting any other communication. All other errors fail silently, as error handling
 * takes up space, and there isn't much that can be done as the bootloader. Instead we'll let either either the watchdog
 * or the communication timer time out and exit the bootloader.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the CAN bus for bootloader communication.
 *
 * @param client_id The client ID to listen for
 * @param channel The configured channel for bootloader communication
 * @return true The CAN bus was successfully initialized
 * @return false CAN bus failed to initialize
 */
bool can_bl_init(unsigned int client_id, unsigned int channel);

/**
 * @brief Attempt to transmit a heartbeat over CAN bus
 */
void can_bl_heartbeat(void);

/**
 * @brief Attempt to receive a message over the bootloader channel
 *
 * @param msg_out Pointer to store received message. Must be at least CAN bus frame size
 * @param len_out Pointer to store length of received message
 * @return true Packet was successfully received into cmd_out
 * @return false No packet was waiting to be received
 */
bool can_bl_try_receive(uint8_t *msg_out, size_t *len_out);

/**
 * @brief Attempt to transmit a message over the bootloader channel
 *
 * @param cmd Array of bytes to send
 * @param len Length of bytes to send. Must be < frame size
 */
void can_bl_transmit(uint8_t *msg, size_t len);

#endif