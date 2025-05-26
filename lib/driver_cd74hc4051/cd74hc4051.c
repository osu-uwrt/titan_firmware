#include "driver/cd74hc4051.h"

#include "hardware/adc.h"
#include "titan/logger.h"

#define NUM_SELECTOR_PINS 3
#define MIN_ADC_PIN 26

#define DIGITAL_LOGIC_HIGH_V 2.0f

uint s_pins[NUM_SELECTOR_PINS];
uint adc_num;

float multiplexer_decode_analog(uint num) {
    // Choose input by setting selector pins
    for (int i = 0; i < NUM_SELECTOR_PINS; i++) {
        gpio_put(s_pins[i], num & (1 << i));
    }

    // Read input, scale to 3V3 and report
    adc_select_input(adc_num);
    return adc_read() * (3.3f / (1 << 12));
}

bool multiplexer_decode_digital(uint num) {
    return multiplexer_decode_analog(num) > DIGITAL_LOGIC_HIGH_V;
}

void multiplexer_init(uint read_pin, uint s0_pin, uint s1_pin, uint s2_pin) {
    if (read_pin < MIN_ADC_PIN) {
        LOG_ERROR("Multiplexer data pin must be connected to an ADC. Pin %u is not", read_pin);
        LOG_ERROR("Multiplexer setup failed");
        return;
    }

    adc_init();
    adc_gpio_init(read_pin);
    adc_num = read_pin - MIN_ADC_PIN;  // Shift to [0, 3] ADC number

    s_pins[0] = s0_pin;
    s_pins[1] = s1_pin;
    s_pins[2] = s2_pin;

    // GPIO setup boilerplate. Makes selection 0 by default
    for (int i = 0; i < NUM_SELECTOR_PINS; i++) {
        gpio_init(s_pins[i]);
        gpio_put(s_pins[i], 0);
        gpio_set_dir(s_pins[i], GPIO_OUT);
    }
}
