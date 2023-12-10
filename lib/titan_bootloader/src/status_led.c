#include "status_led.h"

#include "hardware/gpio.h"

#if defined(STATUS_LEDR_PIN) && defined(STATUS_LEDG_PIN) && defined(STATUS_LEDB_PIN)

#define RGB_MASK ((1 << STATUS_LEDR_PIN) | (1 << STATUS_LEDG_PIN) | (1 << STATUS_LEDB_PIN))

void status_led_init(void) {
    // Initialize LED at full brightness
    gpio_init_mask(RGB_MASK);
    gpio_set_mask(RGB_MASK);
    gpio_set_dir_out_masked(RGB_MASK);
}

void status_led_set(enum status_led_state state) {
    switch (state) {
    case LED_OFF:
    default:
        // Turn LED Off
        gpio_set_mask(RGB_MASK);
        break;

    case LED_BOOT_DELAY:
        // Turn all diodes on
        gpio_clr_mask(RGB_MASK);
        break;

    case LED_BL_NORMAL:
        // RGB set to cyan
        gpio_put(STATUS_LEDR_PIN, 1);
        gpio_put(STATUS_LEDG_PIN, 0);
        gpio_put(STATUS_LEDB_PIN, 0);
        break;

    case LED_BL_FLASH:
        // RGB set to blue
        gpio_put(STATUS_LEDR_PIN, 1);
        gpio_put(STATUS_LEDG_PIN, 1);
        gpio_put(STATUS_LEDB_PIN, 0);
        break;

    // Set red LED to low brightness (using internal pull resistor)
    // Hardware fail will be last state, so no need to set back to output on other states
    case LED_HW_FAIL:
        gpio_set_mask(RGB_MASK);
        gpio_set_dir(STATUS_LEDR_PIN, 0);
        break;
    }
}

#elif defined(PICO_DEFAULT_LED_PIN)

void status_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

void status_led_set(enum status_led_state state) {
    switch (state) {
    // Turn LED On
    case LED_BOOT_DELAY:
    case LED_BL_FLASH:
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        break;

    // Turn LED Off
    case LED_OFF:
    case LED_BL_NORMAL:
    default:
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        break;

    // Set LED to low brightness (using internal pull resistor)
    // Hardware fail will be last state, so no need to set back to output on other states
    case LED_HW_FAIL:
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_IN);
        gpio_pull_up(PICO_DEFAULT_LED_PIN);
        break;
    }
}

#else

#warning No LED pins defined for bootloader

void status_led_init(void) {}
void status_led_set(enum status_led_state state) {
    (void) state;
}

#endif
