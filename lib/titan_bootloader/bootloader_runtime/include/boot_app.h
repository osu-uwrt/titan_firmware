#ifndef _BOOT_APP_H
#define _BOOT_APP_H

#include <stdbool.h>

#define WATCHDOG_BOOTLOADER_NON_REBOOT_MAGIC 0x3e2fea55

/**
 * @brief Attempt to boot the application image in flash.
 *
 * If a valid boot2 image is present in FLASH_APP, this will not return.
 *
 * @param mark_watchdog_reset True if scratch[4] should be filled with WATCHDOG_NON_REBOOT_MAGIC, False if filled with
 * WATCHDOG_BOOTLOADER_NON_REBOOT_MAGIC
 */
void boot_app_attempt(bool mark_watchdog_reset);

#endif
