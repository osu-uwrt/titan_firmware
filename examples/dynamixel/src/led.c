#include <stdbool.h>
#include <rcl/rcl.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "led.h"

/**
 * @brief Rate at which the normal LED value and fault value alternates between
 */
#define LED_TOGGLE_RATE_MS 500

#define LED_BRIGHTNESS_STEPS 32
#define LED_FREQUENCY_HZ 1000
#define LED_LVL_ON LED_BRIGHTNESS_STEPS
#define LED_LVL_OFF 0
#define LED_LVL_YELLOW 7
#define set_led_pin(pin, val)  pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), val)

#ifdef RASPBERRYPI_PICO
#define STATUS_LEDR_PIN 10
#define STATUS_LEDG_PIN 11
#define STATUS_LEDB_PIN 12
#endif

static struct led_status {
    bool initialized;   // If this is false, the PWM hardware hasn't been setup yet
    bool fault;
    bool network_online;
    bool ros_connected;
    bool killswitch;
} status = {.initialized = false};

void led_init() {
    pwm_config config = pwm_get_default_config();

    // Set clkdiv to tick once per millisecond (assert clock is disible cleanly into ms)
    // and for the cycle to rollover after blink cycle ms
    pwm_config_set_clkdiv(&config, ((float)clock_get_hz(clk_sys)) / (LED_FREQUENCY_HZ * LED_BRIGHTNESS_STEPS));
    pwm_config_set_wrap(&config, LED_BRIGHTNESS_STEPS-1);

    // Invert the output polarity as this is an RGB led so it will have inverted logic levels
    pwm_config_set_output_polarity(&config, true, true);

    // Configure PWM hardware
    uint led_pins[] = {STATUS_LEDR_PIN, STATUS_LEDG_PIN, STATUS_LEDB_PIN};
    uint32_t initialized_slices = 0;

    for (unsigned int i = 0; i < sizeof(led_pins) / sizeof(*led_pins); i++){
        uint pin = led_pins[i];
        uint slice_num = pwm_gpio_to_slice_num(pin);

        // Initialize slice if needed
        if (!(initialized_slices & (1<<slice_num))) {
            initialized_slices |= (1<<slice_num);

            pwm_init(slice_num, &config, false);
        }

        // Initialize channel and pin
        set_led_pin(pin, 0);
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }

    // Set the PWM to the LED state
    led_update_pins();

    // Finally enable all the configured slices
    int slice_num = 0;
    while (initialized_slices != 0){
        if (initialized_slices & 1){
            pwm_set_enabled(slice_num, true);
        }
        slice_num++;
        initialized_slices >>= 1;
    }

    status.initialized = true;
}

void led_update_pins() {
    if (!status.initialized)
        return;

    // Color Calculation:
    // 1. Kill switch inserted: Blue
    // 2. Safety Initialized: Green
    // 3. CAN Online: Yellow
    // 4. CAN Offline: Red
    // If a fault occurs, it will alternate between the color above and red

    // If an interrupt fires and this is called recursively, this ensures the value is recomputed
    static volatile bool modified = true;
    do {
        modified = false;

        // Compute blinking if a fault is present
        uint32_t value = (to_ms_since_boot(get_absolute_time()) % (LED_TOGGLE_RATE_MS*2));

        if (status.fault && value >= LED_TOGGLE_RATE_MS) {
            set_led_pin(STATUS_LEDR_PIN, LED_LVL_ON);
            set_led_pin(STATUS_LEDG_PIN, LED_LVL_OFF);
            set_led_pin(STATUS_LEDB_PIN, LED_LVL_OFF);
        }
        else {
            uint16_t r = LED_LVL_OFF;
            uint16_t g = LED_LVL_OFF;
            uint16_t b = LED_LVL_OFF;

            if(status.killswitch) {
                b = LED_LVL_ON;
            } else if(status.ros_connected) {
                g = LED_LVL_ON;
            } else if(status.network_online) {
                r = LED_LVL_ON;
                g = LED_LVL_YELLOW;
            } else {
                r = LED_LVL_ON;
            }

            set_led_pin(STATUS_LEDR_PIN, r);
            set_led_pin(STATUS_LEDG_PIN, g);
            set_led_pin(STATUS_LEDB_PIN, b);
        }
    } while (modified);
    modified = true;
}

void led_fault_set(bool value) {
    status.fault = value;

    if (status.initialized)
        led_update_pins();
}

void led_network_online_set(bool value) {
    status.network_online = value;

    if (status.initialized)
        led_update_pins();
}

void led_ros_connected_set(bool value) {
    status.ros_connected = value;

    if (status.initialized)
        led_update_pins();
}

void led_killswitch_set(bool value) {
    status.killswitch = value;

    if (status.initialized)
        led_update_pins();
}
