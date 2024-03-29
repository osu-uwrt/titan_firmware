#include <string.h>
#include <stdlib.h>
#include "pico.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/watchdog.h"

#include "boot_app.h"
#include "crc32.h"

extern void __boot2_start__(void);
extern void __boot_trampoline_entry__(void);
extern const void __boot_trampoline_end__;
extern const void __boot_trampoline_source__;

extern const void __flash_app;

uint8_t app_boot2_copyout[FLASH_PAGE_SIZE] __attribute__((aligned(FLASH_PAGE_SIZE)));

void boot_app_attempt(bool mark_watchdog_reset) {
    uint32_t flash_app_offset = ((uintptr_t)&__flash_app) - XIP_BASE;
    size_t boot2_crc_offset = sizeof(app_boot2_copyout) - sizeof(uint32_t);

    // Read first page of application flash (application boot2)
    flash_read(flash_app_offset, app_boot2_copyout, sizeof(app_boot2_copyout));

    // Check crc32 (all of first page except for last word)
    uint32_t computed_crc32 = crc32_compute(app_boot2_copyout, boot2_crc_offset);
    uint32_t expected_crc32 = *(uint32_t*)(&app_boot2_copyout[boot2_crc_offset]);

    // Check if boot2 has a valid CRC
    if (computed_crc32 == expected_crc32) {
        // If so boot the image

        // We need to copy out a boot trampoline to a different area of RAM (at the bottom of the stack) as we're executing in XIP cache
        // This needs to be enabled before launching the application, which we can't do here
        size_t boot_trampoline_size = ((uintptr_t)&__boot_trampoline_end__) - ((uintptr_t)&__boot_trampoline_entry__);
        uintptr_t boot_trampoline_offset = ((uintptr_t)&__boot_trampoline_source__) - XIP_BASE;
        hard_assert(boot_trampoline_offset % FLASH_PAGE_SIZE == 0);
        hard_assert(boot_trampoline_size <= FLASH_PAGE_SIZE);
        flash_read(boot_trampoline_offset, (uint8_t*)&__boot_trampoline_entry__, FLASH_PAGE_SIZE);

        // Fill up the watchdog registers if we shouldn't be reporting a watchdog reset
        // Note that if it should, we already have the NON_REBOOT_MAGIC present since a watchdog is already running
        if (!mark_watchdog_reset) {
            // We'll still put a magic value, so in the event the watchdog we still have running fires, it can be
            // detected and the next time it enters it'll be marked as a watchdog reset
            watchdog_hw->scratch[4] = WATCHDOG_BOOTLOADER_NON_REBOOT_MAGIC;

        }

        // Call the trampoline. This shouldn't return
        __boot_trampoline_entry__();
        exit(1);
    }

    // If we failed, then just return to continue the boot process
}