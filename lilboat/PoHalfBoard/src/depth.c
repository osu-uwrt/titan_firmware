
#include "pico/stdlib.h"
#include "depth.h"
#include "hardware/adc.h"
#include "titan/logger.h"
#include <math.h>

#define RP2040_BASE_ADC_PIN 26
#define ADC0_INPUT_NUM (ADC0_PIN - RP2040_BASE_ADC_PIN)
#define ADC1_INPUT_NUM (ADC1_PIN - RP2040_BASE_ADC_PIN)

#define ADC_MIN_DEPTH 0.0
#define ADC_MAX_DEPTH 999.0

void depth_init() {
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_gpio_init(ADC1_PIN);
}

float depth_adc_read(int adc) {
    switch(adc) {
        case 0:
            adc_select_input(ADC0_INPUT_NUM);
            break;
        case 1:
            adc_select_input(ADC1_INPUT_NUM);
            break;
        default:
            LOG_ERROR("cannot read depth ADC: invalid ADC number %d", adc);
            return NAN;
    }

    const float r1 = 1.0;
    const float r2 = 1.0;
    float reading = ((float) adc_read()) / (1 << 12);
    reading = (reading * (r1 + r2)) / r2;

    return MIN(ADC_MAX_DEPTH, MAX(ADC_MIN_DEPTH, reading));
}
