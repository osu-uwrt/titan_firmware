#include "bl_interface.h"
#include "bl_server.h"
#include "boot_app.h"
#include "dbg_uart.h"
#include "status_led.h"

#include "hardware/watchdog.h"
#include "pico/time.h"
#include "titan/version.h"

#define WATCHDOG_TIMEOUT_MS 3000
#define LINK_DELAY_MS 5000
#define BOOT_DELAY_MS 500
#define BOOTLOADER_TIMEOUT_SEC 30

void tick_led(void) {
    static absolute_time_t next_update = { 0 };
    static unsigned int led_state = 0;
    if (time_reached(next_update)) {
        // Flash LED in a cool pattern
        if ((led_state + 1) % 2) {
            status_led_set(LED_BL_FLASH);
        }
        else {
            status_led_set(LED_BL_NORMAL);
        }

        led_state++;
        if (led_state == 10) {
            led_state = 0;
            next_update = make_timeout_time_ms(750);
        }
        else {
            next_update = make_timeout_time_ms(75);
        }
    }
}

void tick_heartbeat(void) {
    static absolute_time_t next_update = { 0 };
    if (time_reached(next_update)) {
        next_update = make_timeout_time_ms(500);
        bl_interface_heartbeat();
    }
}

void run_bootloader(void) {
    dbg_uart_puts("Entering bootloader...");
    status_led_set(LED_BL_NORMAL);

    // Exit bootloader after a given amount of time, forcing a reset
    // In the event the board enters bootloader for whatever reason, it keeps it from being stuck offline
    // Worst case it'll reboot into the main application within a minute
    absolute_time_t bootloader_timeout = make_timeout_time_ms(BOOTLOADER_TIMEOUT_SEC * 1000);

    while (!time_reached(bootloader_timeout) && !bl_server_should_reboot()) {
        watchdog_update();

        tick_led();
        tick_heartbeat();
        if (bl_server_tick()) {
            // Postpone timeout if we've gotten communication from the host
            bootloader_timeout = make_timeout_time_ms(BOOTLOADER_TIMEOUT_SEC * 1000);
        }
    }

    // Don't try to return after exiting bootloader, just exit
    // This is so that in the event the device was reflashed, it'll get a clean start
    watchdog_hw->scratch[0] = 0x1035000;
    watchdog_hw->scratch[1] = 0x7193006;
    watchdog_reboot(0, 0, 0);
}

int main(void) {
    // First step, before enabling the watchdog (which clears scratch[4]), check if bootloader magic set
    bool enter_bootloader = false;
    bool notify_watchdog_reset = false;
    if (watchdog_hw->scratch[4] == 0xb00710ad) {
        enter_bootloader = true;
        watchdog_hw->scratch[4] = 0;
    }
    else if ((watchdog_caused_reboot() && watchdog_hw->scratch[4] == WATCHDOG_BOOTLOADER_NON_REBOOT_MAGIC) ||
             watchdog_enable_caused_reboot()) {
        notify_watchdog_reset = true;
    }

    // Next enable watchdog before anything else
    // In the event something in the bootloader crashes, or the app image is corrupted after branching,
    // we can go back into the bootloader. Even if the main image is bootlooping, the boot delay allows recovery
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    // Initialize Status LED
    status_led_init();

    // Send out bootloader version over UART
    dbg_uart_init();
    dbg_uart_puts("");
    dbg_uart_puts(FULL_BUILD_TAG);

    // Initialize CAN Bus
    if (!bl_interface_init()) {
        dbg_uart_puts("Failed to initialize bootloader interface!");

        // If CAN fails to bring up, just abort (and change the LED to show that it messed up)
        status_led_set(LED_HW_FAIL);
        busy_wait_ms(BOOT_DELAY_MS);

        // Try to boot an image, if this fails, just exit
        boot_app_attempt(notify_watchdog_reset);
        return 0;
    }

    bl_server_init();

    // Wait for link before doing anything else
    uint64_t poll_start = time_us_64();
    while (((time_us_64() - poll_start) < LINK_DELAY_MS * 1000)) {
        if (bl_interface_check_online()) {
            break;
        }
        watchdog_update();
    }
    bl_interface_notify_boot();

    // Then check if we should enter bootloader
    if (bl_interface_check_online()) {
        poll_start = time_us_64();
        while (((time_us_64() - poll_start) < BOOT_DELAY_MS * 1000) && !enter_bootloader) {
            if (bl_server_check_for_magic_packet()) {
                enter_bootloader = true;
            }
            watchdog_update();
        }
    }

    // If we aren't supposed to enter bootloader, try to boot the application image
    if (!enter_bootloader) {
        watchdog_update();
        boot_app_attempt(notify_watchdog_reset);
        dbg_uart_puts("Failed to boot application! Falling back to bootloader");
    }

    // Fallthrough into bootloader
    run_bootloader();

    return 0;
}
