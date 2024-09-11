#include "ledc_driver.h"

#include "ledc_commands.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "titan/debug.h"

#define CONTROLLER_WATCHDOG_PERIOD_MS 3

#define LED_UPDATE_INTERVAL_MS 50
#define LED_TIMER_PERIOD_TICKS                                                                                         \
    120  // Note this should be less than 256 to avoid multiplication overflows and divisible by 2
#define LED_FAST_FLASH_PERIOD 6   // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
#define LED_SLOW_FLASH_PERIOD 40  // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
// Note breath uses LED_TIMER_PERIOD

#define LED_FLASH_PULSE_PERIOD 6  // Note this should be less than 256 to avoid multiplication overflows
#define LED_FLASH_PULSE_COUNT 2

#define OPERATING_TEMPERATURE_MAX_C 50

static repeating_timer_t status_update_timer, controller_watchdog_timer;

// LED state config
volatile bool led_enabled;
enum status_mode led_mode;
uint8_t red_target;
uint8_t green_target;
uint8_t blue_target;
uint led_timer;

// Flash State Config
volatile bool flash_active;  // Voltaile as this is used to protect the flashing state, rather than disabling interrupts
uint flash_timer;
uint flash_count;
uint8_t red_flash_target;
uint8_t green_flash_target;
uint8_t blue_flash_target;

static bool __time_critical_func(update_led_status)(__unused repeating_timer_t *rt) {
    uint red, green, blue;

    if (led_mode == MODE_SOLID) {
        red = red_target;
        green = green_target;
        blue = blue_target;
    }
    else if (led_mode == MODE_FAST_FLASH || led_mode == MODE_SLOW_FLASH) {
        uint flash_period = (led_mode == MODE_FAST_FLASH ? LED_FAST_FLASH_PERIOD : LED_SLOW_FLASH_PERIOD);
        if (led_timer % flash_period < (flash_period / 2)) {
            red = red_target;
            green = green_target;
            blue = blue_target;
        }
        else {
            // Set blank
            red = blue = green = 0;
        }
    }
    else if (led_mode == MODE_BREATH) {
        if (led_timer % LED_TIMER_PERIOD_TICKS < (LED_TIMER_PERIOD_TICKS / 2)) {
            // Handle rising fade
            red = (((uint32_t) red_target) * led_timer) / (LED_TIMER_PERIOD_TICKS / 2);
            green = (((uint32_t) green_target) * led_timer) / (LED_TIMER_PERIOD_TICKS / 2);
            blue = (((uint32_t) blue_target) * led_timer) / (LED_TIMER_PERIOD_TICKS / 2);
        }
        else {
            // Handle falling fade
            red = (red_target) -
                  (((uint32_t) red_target) * (led_timer - (LED_TIMER_PERIOD_TICKS / 2)) / (LED_TIMER_PERIOD_TICKS / 2));
            green = (green_target) - (((uint32_t) green_target) * (led_timer - (LED_TIMER_PERIOD_TICKS / 2)) /
                                      (LED_TIMER_PERIOD_TICKS / 2));
            blue = (blue_target) - (((uint32_t) blue_target) * (led_timer - (LED_TIMER_PERIOD_TICKS / 2)) /
                                    (LED_TIMER_PERIOD_TICKS / 2));
        }
        // Square it to make fading look smoother
        red = (((uint32_t) red) * ((uint32_t) red)) >> 8;
        green = (((uint32_t) green) * ((uint32_t) green)) >> 8;
        blue = (((uint32_t) blue) * ((uint32_t) blue)) >> 8;
    }
    else {
        // Set blank
        red = blue = green = 0;
    }
    led_timer = (led_timer + 1) % LED_TIMER_PERIOD_TICKS;  // Tick the timer

    // Handle quick flash requests
    if (flash_active) {
        // First compute color for this round of flashing
        if (flash_timer % LED_FLASH_PULSE_PERIOD < (LED_FLASH_PULSE_PERIOD / 2)) {
            // Handle rising fade
            red = (((uint32_t) red_flash_target) * flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
            green = (((uint32_t) green_flash_target) * flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
            blue = (((uint32_t) blue_flash_target) * flash_timer) / (LED_FLASH_PULSE_PERIOD / 2);
        }
        else {
            // Handle falling fade
            red = (red_flash_target) - (((uint32_t) red_flash_target) * (flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) /
                                        (LED_FLASH_PULSE_PERIOD / 2));
            green =
                (green_flash_target) - (((uint32_t) green_flash_target) * (flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) /
                                        (LED_FLASH_PULSE_PERIOD / 2));
            blue = (blue_flash_target) - (((uint32_t) blue_flash_target) *
                                          (flash_timer - (LED_FLASH_PULSE_PERIOD / 2)) / (LED_FLASH_PULSE_PERIOD / 2));
        }

        // Then compute the next timer and count values
        flash_timer++;
        if (flash_timer >= LED_FLASH_PULSE_PERIOD) {
            flash_timer = 0;
            flash_count++;
            if (flash_count >= LED_FLASH_PULSE_COUNT) {
                flash_active = false;
            }
        }
    }

    // Clear color output if LEDs are disabled
    if (!led_enabled) {
        red = green = blue = 0;
    }

    // Transmit data

    return true;
}

void ledc_init() {
    init_spi_and_gpio();

    // LEDC defines set in ledc_commands.h
    for (uint controller = LEDC1; controller <= LEDC2; controller++) {
        for (uint buck = BUCK1; buck <= BUCK2; buck++) {
            // Sets LEDs to off by default using PWM dimming
            buck_set_control_mode(controller, buck, BUCK_PWM_DIMMING);
            sleep_ms(1);
        }

        controller_clear_watchdog_error(controller);
        controller_enable(controller);
    }

    led_enabled = true;
    led_timer = 0;
    flash_active = false;

    add_repeating_timer_ms(CONTROLLER_WATCHDOG_PERIOD_MS, controller_satisfy_watchdog, NULL,
                           &controller_watchdog_timer);
    hard_assert(add_repeating_timer_ms(LED_UPDATE_INTERVAL_MS, update_led_status, NULL, &status_update_timer));
}
