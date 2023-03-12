#ifndef _BOOT_APP_H
#define _BOOT_APP_H

/**
 * @brief Attempt to boot the application image in flash.
 *
 * If a valid boot2 image is present in FLASH_APP, this will not return.
 */
void boot_app_attempt(void);

#endif