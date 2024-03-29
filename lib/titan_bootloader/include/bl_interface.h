#ifndef BL_INTERFACE_H
#define BL_INTERFACE_H

/**
 * @file bl_interface.h
 * @brief Stripped down interface for bootloader communciation
 *
 * This is an abstract interface which can be implemented by a variety of protocols, mainly CAN bus and Ethernet.
 * This allows for a single bootloader application to be built for a variety of protocols.
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

#define BL_INTERFACE_MAX_PACKET_LEN 8

/**
 * @brief Initialize the communication channel for bootloader communication.
 *
 * @return true The CAN bus was successfully initialized
 * @return false CAN bus failed to initialize
 */
bool bl_interface_init(void);

/**
 * @brief Attempt to transmit a heartbeat over the heartbeat channel
 */
void bl_interface_heartbeat(void);

/**
 * @brief Check if the given interface is online
 */
bool bl_interface_check_online(void);

/**
 * @brief Attempt to notify that the device has booted over the heartbeat channel
 */
void bl_interface_notify_boot(void);

/**
 * @brief Attempt to receive a message over the bootloader channel
 *
 * @param msg_out Pointer to store received message. Must be at BL_INTERFACE_MAX_PACKET_LEN in size
 * @param len_out Pointer to store length of received message
 * @return true Packet was successfully received into cmd_out
 * @return false No packet was waiting to be received
 */
bool bl_interface_try_receive(uint8_t *msg_out, size_t *len_out);

/**
 * @brief Attempt to transmit a message over the bootloader channel
 *
 * @param cmd Array of bytes to send
 * @param len Length of bytes to send. Must be < BL_INTERFACE_MAX_PACKET_LEN
 */
void bl_interface_transmit(uint8_t *msg, size_t len);

#endif
