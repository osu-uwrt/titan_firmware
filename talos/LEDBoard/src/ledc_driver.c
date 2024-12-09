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

static repeating_timer_t status_update_timer, controller_watchdog_timer, depth_monitor_timer, singleton_flash_timer,
    temperature_monitor_timer;

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

// Short singleton flash (vision detections)
volatile bool do_singleton;
uint next_singleton = LOOPS_PER_SINGLETON;
bool is_in_singleton = false;
uint8_t red_singleton_target;
uint8_t green_singleton_target;
uint8_t blue_singleton_target;

// Track the last values set by the driver so they can be restored later
uint8_t last_red;
uint8_t last_green;
uint8_t last_blue;
float last_max_brightness;

// Depth status
volatile bool is_underwater = false;
volatile bool depth_stale = true;
volatile bool got_new_depth = false;

bool em_overtemp = false;
bool high_temp = false;
float curr_al_temp = 0.0f;

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

    // Use brightness dictated by depth management
    float max_brightness = is_underwater && !depth_stale ? WATER_MAX_BRIGHTNESS : BENCH_MAX_BRIGHTNESS;

    // Scale brightness based on temperature
    float temp_adjust =
        (-1.0f / (MAX_OPERATING_TEMPERATURE_C - BASE_OPERATING_TEMPERATURE_C)) * curr_al_temp +
        (MAX_OPERATING_TEMPERATURE_C * 1.0f) / (MAX_OPERATING_TEMPERATURE_C - BASE_OPERATING_TEMPERATURE_C);
    // Clamp to [0, 1]
    temp_adjust = temp_adjust < 0.0f ? 0.0f : temp_adjust;
    temp_adjust = temp_adjust > 1.0f ? 1.0f : temp_adjust;

    // Turn off LEDs if temp exceeds max
    if (em_overtemp)
        max_brightness = 0.0f;

    last_red = red;
    last_green = green;
    last_blue = blue;
    last_max_brightness = max_brightness;

    // Allow singleton flashes every LOOPS_PER_SINGLETON iterations
    if (next_singleton > 0)
        next_singleton--;

    // Transmit data
    led_set_rgb(red, green, blue, max_brightness * temp_adjust);

    return true;
}

static bool monitor_depth(__unused repeating_timer_t *rt) {
    // Set to low brightness if we lose depth
    depth_stale = !got_new_depth;
    got_new_depth = false;

    return true;
}

static bool handle_singleton_flash(__unused repeating_timer_t *rt) {
    if (next_singleton > 0 || !do_singleton)  // Singleton not allowed
        return true;

    if (is_in_singleton) {  // Set back to pre-flash color
        led_set_rgb(last_red, last_green, last_blue, last_max_brightness);
        is_in_singleton = false;
    }
    else {  // Set to flash color, to be reset the next loop
        led_set_rgb(red_singleton_target, green_singleton_target, blue_singleton_target, last_max_brightness);
        is_in_singleton = true;
    }

    // Reset state and counter
    do_singleton = false;
    next_singleton = LOOPS_PER_SINGLETON;

    return true;
}

// Get aluminum board thermistor temps and raise/lower faults accordingly
// Also adjusts peak current based on temp
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

// Configure, set inital states, start repeating timers
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

// Set local persistent driver mode
void led_set(enum status_mode mode, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t prev_interrupts = save_and_disable_interrupts();
    led_timer = 0;
    led_mode = mode;
    red_target = red;
    green_target = green;
    blue_target = blue;
    restore_interrupts(prev_interrupts);
}

// Flash (on kill switch insertion), not persistent
void led_flash(uint8_t red, uint8_t green, uint8_t blue) {
    flash_active = false;
    red_flash_target = red;
    green_flash_target = green;
    blue_flash_target = blue;
    flash_timer = 0;
    flash_count = 0;
    flash_active = true;
}

// Short flash (on vision detection), not persistent
void led_singleton(uint8_t red, uint8_t green, uint8_t blue) {
    do_singleton = false;
    red_singleton_target = red;
    green_singleton_target = green;
    blue_singleton_target = blue;
    do_singleton = true;
}

void led_enable(void) {
    led_enabled = true;
}

void led_disable(void) {
    led_enabled = false;
}

// Use underwater brightness if below threshold
void led_depth_set(float depth) {
    is_underwater = depth < UNDERWATER_MIN_DEPTH;
    got_new_depth = true;
}
