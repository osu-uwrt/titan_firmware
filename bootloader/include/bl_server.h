#ifndef BL_SERVER_H
#define BL_SERVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the bootloader server
 */
void bl_server_init(void);

/**
 * @brief Checks for magic packet to enter bootloader mode
 *
 * @return true if the magic packet was successfully received
 */
bool bl_server_check_for_magic_packet(void);

/**
 * @brief Attempt to process pending requests for the bootloader server
 *
 * @return true if a request was processed
 */
bool bl_server_tick(void);

/**
 * @brief Checks if the client has requested that the MCU reboot
 *
 * @return true if the mcu should reboot
 */
bool bl_server_should_reboot(void);

#endif