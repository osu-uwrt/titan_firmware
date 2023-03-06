#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/time.h"

#include "can_bl_interface.h"

#define RGB_MASK ((1<<STATUS_LEDR_PIN) | (1<<STATUS_LEDG_PIN) | (1<<STATUS_LEDB_PIN))

#define BOOTLOADER_TIMEOUT_SEC 30
#define BOOT_DELAY_MS 500

void tick_led(void) {
    static absolute_time_t next_update = {0};
    static unsigned int led_state = 0;
    if (time_reached(next_update)) {
        gpio_put(STATUS_LEDG_PIN, (led_state+1) % 2);

        led_state++;
        if (led_state == 10) {
            led_state = 0;
            next_update = make_timeout_time_ms(750);
        } else {
            next_update = make_timeout_time_ms(75);
        }
    }
}

void tick_heartbeat(void) {
    static absolute_time_t next_update = {0};
    if (time_reached(next_update)) {
        next_update = make_timeout_time_ms(500);
        can_bl_heartbeat();
    }
}

bool handle_test_protocol(void) {
    uint8_t msg[8];
    size_t size;
    if (can_bl_try_receive(msg, &size)) {
        for (uint i = 0; i < size; i++) {
            msg[i]++;
        }
        can_bl_transmit(msg, size);
        return true;
    } else {
        return false;
    }
}

void run_bootloader(void) {
    // Configure LED pins
    gpio_put(STATUS_LEDR_PIN, 1);
    gpio_put(STATUS_LEDB_PIN, 0);

    absolute_time_t bootloader_timeout = make_timeout_time_ms(BOOTLOADER_TIMEOUT_SEC * 1000);

    while (!time_reached(bootloader_timeout)) {
        tick_led();
        tick_heartbeat();
        if (handle_test_protocol()) {
            bootloader_timeout = make_timeout_time_ms(BOOTLOADER_TIMEOUT_SEC * 1000);
        }
    }

    // Don't try to return after exiting bootloader, just exit
    // This is so that in the event the device was reflashed, it'll get a clean start
    watchdog_reboot(0, 0, 0);
}

int main(void) {
    // Initialize RGB LEDs
    gpio_init_mask(RGB_MASK);
    gpio_clr_mask(RGB_MASK);
    gpio_set_dir_out_masked(RGB_MASK);

    // Initialize CAN Bus
    if (!can_bl_init(1, 1)) {
        // If CAN fails to bring up, just abort (and change the LED to show that it messed up)
        gpio_set_mask(RGB_MASK);
        gpio_set_dir(STATUS_LEDR_PIN, 0);

        busy_wait_ms(BOOT_DELAY_MS);
        return 0;
    }

    bool enter_bootloader = false;
    can_bl_heartbeat();

    // TODO: Print version string to serial console
    // TODO: Enable watchdog
    // TODO: Read watchdog register and clear so we don't infinitely loop

    uint8_t msg[8];
    size_t size;
    while ((time_us_64() < BOOT_DELAY_MS * 1000) && !enter_bootloader) {
        if (can_bl_try_receive(msg, &size)) {
            // TODO: Replace to be the actual check rather than some garbage check
            if (msg[0] > 0) {
                enter_bootloader = true;
            }
        }
    }

    if (enter_bootloader) {
        run_bootloader();
    }

    // Set color code in the event if hangs during the trampoline
    gpio_set_mask(RGB_MASK);
    gpio_set_dir(STATUS_LEDB_PIN, 0);
    return 0;
}