/**
 * @file ledc_driver.c
 * @author Ohio State Underwater Robotic Team (UWRT)
 * @brief Main Implementation File for describing the LED drivers for Talos.
 * @version 0.1
 * @date 2026-09-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ledc_driver.h"

#include "ledc_commands.h"
#include "safety_interface.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/time.h"
#include "titan/debug.h"

//
// Driver timing management
//

#define CONTROLLER_WATCHDOG_PERIOD_MS 3
#define DEPTH_MONITOR_PERIOD_MS 1000
#define TEMPERATURE_MONITOR_PERIOD_MS 1000
#define PEAK_CURRENT_STAGGER_MS 10

#define LED_UPDATE_INTERVAL_MS 50
#define LED_TIMER_PERIOD_TICKS                                                                                         \
    120  // Note this should be less than 256 to avoid multiplication overflows and divisible by 2
#define LED_FAST_FLASH_PERIOD 6   // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
#define LED_SLOW_FLASH_PERIOD 40  // This should be divisible by 2 and LED_TIMER_PERIOD_TICKS divisible by this
// Note breath uses LED_TIMER_PERIOD

#define LED_FLASH_PULSE_PERIOD 6  // Note this should be less than 256 to avoid multiplication overflows
#define LED_FLASH_PULSE_COUNT 2

#define SINGLETON_FLASH_PERIOD_MS 10
#define LOOPS_PER_SINGLETON 5  // Number of main loops until the next singleton is allowed

//
// Depth brightness adjustment
//

#define WATER_MAX_BRIGHTNESS 1.0f
#define BENCH_MAX_BRIGHTNESS 0.01f
#define UNDERWATER_MIN_DEPTH -0.05f

//
// Temperature safety
//

#define BASE_OPERATING_TEMPERATURE_C 30
#define MAX_OPERATING_TEMPERATURE_C 50

#define NORMAL_OPERATION_PEAK_CURRENT 45  // See LED controller datasheet

// Abstraction of internal implementation of squaring an unsigned integer
#define SQUARE(x) _led_square_value(x)

static repeating_timer_t status_update_timer, controller_watchdog_timer, depth_monitor_timer, singleton_flash_timer,
    temperature_monitor_timer;

typedef struct led_rgb_values_saved_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_rgb_values_saved_t;

typedef struct led_rgb_values_runtime_t {
    uint red;
    uint green;
    uint blue;
} led_rgb_values_runtime_t;

// LED state config
volatile bool led_enabled;
enum status_mode led_mode;
static led_rgb_values_saved_t *rgb_target;

uint led_timer;

// Flash State Config
volatile bool flash_active;  // Volatile as this is used to protect the flashing state, rather than disabling interrupts
uint flash_timer;
uint flash_count;
static led_rgb_values_saved_t *rgb_flash_target;

// Short singleton flash (vision detections)
volatile bool do_singleton;
uint next_singleton = LOOPS_PER_SINGLETON;
bool is_in_singleton = false;
static led_rgb_values_saved_t *rgb_singleton_target;

// Track the last values set by the driver so they can be restored later
static led_rgb_values_saved_t *rgb_last;
float last_max_brightness;

// Depth status
volatile bool is_underwater = false;
volatile bool depth_stale = true;
volatile bool got_new_depth = false;

bool em_overtemp = false;
bool high_temp = false;
float curr_al_temp = 0.0f;

/**
 * @brief Object Function Wrapper Macro.
 *
 * @note inline is debatable since compiler will (pretty likely) ignore it,
 *       especially if the function itself somehow becomes more complex or looped.
 */
#define led_force_inline static inline

/**
 * @brief Executes the squaring of parameter x and returns it.
 *
 * @note Although pow() does exist, doing integer promotion and bit shifting often is more efficient and faster to
 *       execute.
 *
 * @param x The number that you want to have squared.
 * @return (uint) The value that will be the result of that squaring.
 */
led_force_inline uint _led_square_value(uint x) {
    return ((((uint32_t) x) * ((uint32_t) x)) >> 8);
}

/**
 * @brief Calculates and returns the new color value based on the color rising and the breathe mode of the LED driver.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param color The current color value that you have.
 * @param timer The value that will be taken as the time that you want to find the new color at.
 *
 * @return uint The next color that would be given the ```timer```'s argued time and the argued ```color``` value.
 */
led_force_inline uint _calculate_breathe_rising_fade(uint8_t color, uint timer) {
    return (((uint32_t) color) * timer) / (LED_TIMER_PERIOD_TICKS / 2);
}

/**
 * @brief Calculates and returns the new color value based on the color rising and the flash mode of the LED driver.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param color The current color value that you have.
 * @param timer The value that will be taken as the time that you want to find the new color at.
 *
 * @return uint The next color that would be given the ```timer```'s argued time and the argued ```color``` value.
 */
led_force_inline uint _calculate_flash_rising_fade(uint8_t color, uint timer) {
    return (((uint32_t) color) * timer / (LED_FLASH_PULSE_PERIOD / 2));
}

/**
 * @brief Calculates and returns the new color value based on the color fading and the breathe mode of the LED driver.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param color The current color value that you have.
 * @param timer The value that will be taken as the time that you want to find the new color at.
 *
 * @return uint The next color that would be given the ```timer```'s argued time and the argued ```color``` value.
 */
led_force_inline uint _calculate_breathe_falling_fade(uint8_t color, uint timer) {
    return (color) - (((uint32_t) color) * (timer - (LED_TIMER_PERIOD_TICKS / 2)) / (LED_TIMER_PERIOD_TICKS / 2));
}

/**
 * @brief Calculates and returns the new color value based on the color fading and the fade mode of the LED driver.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param color The current color value that you have.
 * @param timer The value that will be taken as the time that you want to find the new color at.
 *
 * @return uint The next color that would be given the ```timer```'s argued time and the argued ```color``` value.
 */
led_force_inline uint _calculate_flash_falling_fade(uint8_t color, uint timer) {
    return (color) - (((uint32_t) color) * (timer - (LED_FLASH_PULSE_PERIOD / 2)) / (LED_FLASH_PULSE_PERIOD / 2));
}

/**
 * @brief Updates the ```obj``` pointer variable to have its member variables be equal to the values argued as red,
 *        green, and blue.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param obj[in,out] The pointer of the rgb saved values that will be modified.
 * @param red The new red color value.
 * @param green The new green color value.
 * @param blue The new blue color value.
 */
led_force_inline void _update_saved_rgb_values(led_rgb_values_saved_t *obj, uint8_t red, uint8_t green, uint8_t blue) {
    obj->red = red;
    obj->green = green;
    obj->blue = blue;
}

/**
 * @brief Predicts the temperature factor that will affect the brightness of the LEDs.
 *
 * @note Internal function. The exact implementation is subject to change if deemed worthy.
 *
 * @param current_temperature The current temperature that is recorded from the Aluminum LED board.
 *
 * @return float The temperature factor that will affect the lights, unclamped.
 */
led_force_inline float _led_calculate_new_tmp_factor(float current_temperature) {
    return (float) (-1.0f / (MAX_OPERATING_TEMPERATURE_C - BASE_OPERATING_TEMPERATURE_C)) * current_temperature +
           (MAX_OPERATING_TEMPERATURE_C * 1.0f) / (MAX_OPERATING_TEMPERATURE_C - BASE_OPERATING_TEMPERATURE_C);
}

/**
 * @brief Callback function that overlooks and handles LED RGB values based of the led_mode, the definitions of the
 *        different rising and falling methods, the current temperature of the LED aluminum board, and the current
 *        underwater depth.
 *
 * @note param ```rt``` is unused.
 *
 * @return Always returns @c true to indicate a successful packet transfer.
 */
static bool __time_critical_func(update_led_status)(__unused repeating_timer_t *rt) {
    uint red, green, blue;

    if (led_mode == MODE_SOLID) {
        red = rgb_target->red;
        green = rgb_target->green;
        blue = rgb_target->blue;
    }
    else if (led_mode == MODE_FAST_FLASH || led_mode == MODE_SLOW_FLASH) {
        uint flash_period = (led_mode == MODE_FAST_FLASH ? LED_FAST_FLASH_PERIOD : LED_SLOW_FLASH_PERIOD);
        if (led_timer % flash_period < (flash_period / 2)) {
            red = rgb_target->red;
            green = rgb_target->green;
            blue = rgb_target->blue;
        }
        else {
            // Set blank
            red = green = blue = 0;
        }
    }
    else if (led_mode == MODE_BREATH) {
        if (led_timer % LED_TIMER_PERIOD_TICKS < (LED_TIMER_PERIOD_TICKS / 2)) {
            red = _calculate_breathe_rising_fade(rgb_target->red, led_timer);
            green = _calculate_breathe_rising_fade(rgb_target->green, led_timer);
            blue = _calculate_breathe_rising_fade(rgb_target->blue, led_timer);
        }
        else {
            red = _calculate_breathe_falling_fade(rgb_target->red, led_timer);
            green = _calculate_breathe_falling_fade(rgb_target->green, led_timer);
            blue = _calculate_breathe_falling_fade(rgb_target->blue, led_timer);
        }
        red = SQUARE(red);
        green = SQUARE(green);
        blue = SQUARE(blue);
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
            red = _calculate_flash_rising_fade(rgb_flash_target->red, flash_timer);
            green = _calculate_flash_rising_fade(rgb_flash_target->green, flash_timer);
            blue = _calculate_flash_rising_fade(rgb_flash_target->blue, flash_timer);
        }
        else {
            red = _calculate_flash_falling_fade(rgb_flash_target->red, flash_timer);
            green = _calculate_flash_falling_fade(rgb_flash_target->green, flash_timer);
            blue = _calculate_flash_falling_fade(rgb_flash_target->blue, flash_timer);
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

    // Use brightness dictated by depth management
    float max_brightness = is_underwater && !depth_stale ? WATER_MAX_BRIGHTNESS : BENCH_MAX_BRIGHTNESS;

    // Scale brightness based on temperature
    float temp_adjust = _led_calculate_new_tmp_factor(curr_al_temp);
    // Clamp to [0, 1]
    temp_adjust = temp_adjust < 0.0f ? 0.0f : temp_adjust;
    temp_adjust = temp_adjust > 1.0f ? 1.0f : temp_adjust;

    // Turn off LEDs if temp exceeds max
    if (em_overtemp)
        max_brightness = 0.0f;

    _update_saved_rgb_values(rgb_last, red, green, blue);

    last_max_brightness = max_brightness;

    // Allow singleton flashes every LOOPS_PER_SINGLETON iterations
    if (next_singleton > 0)
        next_singleton--;

    // Transmit data
    led_set_rgb(red, green, blue, max_brightness * temp_adjust);

    return true;
}

/**
 * @brief Callback function to monitor the current depth and set to low brightness if we lose depth.
 *
 * @note param ```rt``` is unused.
 *
 * @return Always returns @c true to indicate a successful packet transfer.
 */
static bool monitor_depth(__unused repeating_timer_t *rt) {
    // Set to low brightness if we lose depth
    depth_stale = !got_new_depth;
    got_new_depth = false;

    return true;
}

/**
 * @brief Callback function to set the singleton flash for the LED Board.
 *
 * @note param ```rt``` is unused.
 *
 * @return Always returns @c true to indicate a successful packet transfer.
 */
static bool handle_singleton_flash(__unused repeating_timer_t *rt) {
    if (next_singleton > 0 || !do_singleton)  // Singleton not allowed
        return true;

    if (is_in_singleton) {  // Set back to pre-flash color
        led_set_rgb(rgb_last->red, rgb_last->green, rgb_last->blue, last_max_brightness);
        is_in_singleton = false;
    }
    else {  // Set to flash color, to be reset the next loop
        led_set_rgb(rgb_singleton_target->red, rgb_singleton_target->green, rgb_singleton_target->blue,
                    last_max_brightness);
        is_in_singleton = true;
    }

    // Reset state and counter
    do_singleton = false;
    next_singleton = LOOPS_PER_SINGLETON;

    return true;
}

/**
 * @brief Callback function for getting the current temperature of the Aluminum LED board.
 *
 * @note Get aluminum board thermistor temps and raise/lower faults accordingly
 *       Also adjusts peak current based on temp.
 *
 * @note param ```rt``` is unused.
 *
 * @return Always returns @c true to indicate a successful packet transfer.
 */
static bool monitor_temperature(__unused repeating_timer_t *rt) {
    curr_al_temp = al_read_temp();

    if (curr_al_temp > MAX_OPERATING_TEMPERATURE_C) {
        safety_raise_fault_with_arg(FAULT_LED_OVERTEMP, curr_al_temp);
        em_overtemp = true;
    }
    else {
        safety_lower_fault(FAULT_LED_OVERTEMP);
        em_overtemp = false;
    }

    return true;
}

void ledc_init() {
    init_spi_and_gpio();
    register_canmore_commands();
    led_set_rgb(0, 0, 0, 1023);  // Set LEDs off before enabling them

    // LEDC defines set in ledc_commands.h
    for (uint controller = LEDC1; controller <= LEDC2; controller++) {
        for (uint buck = BUCK1; buck <= BUCK2; buck++) {
            buck_set_control_mode(controller, buck, BUCK_PWM_DIMMING);
            buck_set_peak_current(controller, buck, NORMAL_OPERATION_PEAK_CURRENT);
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
    add_repeating_timer_ms(SINGLETON_FLASH_PERIOD_MS, handle_singleton_flash, NULL, &singleton_flash_timer);
    add_repeating_timer_ms(DEPTH_MONITOR_PERIOD_MS, monitor_depth, NULL, &depth_monitor_timer);
    hard_assert(
        add_repeating_timer_ms(TEMPERATURE_MONITOR_PERIOD_MS, monitor_temperature, NULL, &temperature_monitor_timer));
}

void led_set(status_mode mode, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t prev_interrupts = save_and_disable_interrupts();
    led_timer = 0;
    led_mode = mode;
    _update_saved_rgb_values(rgb_target, red, green, blue);
    restore_interrupts(prev_interrupts);
}

void led_flash(uint8_t red, uint8_t green, uint8_t blue) {
    flash_active = false;
    _update_saved_rgb_values(rgb_flash_target, red, green, blue);
    flash_timer = 0;
    flash_count = 0;
    flash_active = true;
}

void led_singleton(uint8_t red, uint8_t green, uint8_t blue) {
    do_singleton = false;
    _update_saved_rgb_values(rgb_singleton_target, red, green, blue);
    do_singleton = true;
}

void led_depth_set(float depth) {
    is_underwater = depth < UNDERWATER_MIN_DEPTH;
    got_new_depth = true;
}
