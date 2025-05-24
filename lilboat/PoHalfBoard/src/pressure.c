#include "pressure.h"

#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "titan/logger.h"

#include <math.h>

#define RP2040_BASE_ADC_PIN 26
#define ADC0_INPUT_NUM (ADC0_PIN - RP2040_BASE_ADC_PIN)
#define ADC1_INPUT_NUM (ADC1_PIN - RP2040_BASE_ADC_PIN)

#define ADC_MIN_PRESSURE 0.0
#define ADC_MAX_PRESSURE 1600.0

void pressure_init() {
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_gpio_init(ADC1_PIN);
}

float pressure_read_adc(int adc) {
    switch (adc) {
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

    const float max_voltage = 5.0 * (2.2 / 3.2);  // 5V signal that goes into a voltage divider
    const float offset = 0.0;
    const float scale = 1600.0f;
    float reading = ((float) adc_read()) / (1 << 12);
    reading *= max_voltage;
    reading -= offset;
    reading *= scale;

    return MIN(ADC_MAX_PRESSURE, MAX(ADC_MIN_PRESSURE, reading));
}
