#include "pico/stdlib.h"
#include "analog_io.h"
#include "hardware/adc.h"

#define ADC_NUM_PORT_MEAS 0
#define ADC_PIN_PORT_MEAS 26
#define ADC_NUM_STBD_MEAS 1
#define ADC_PIN_PORT_STBD 27

#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

#define ADC_TIMER_TIME 10

static uint16_t adc_values[2];
static int current_adc_channel;

void switch_to_next_channel() { 
    current_adc_channel = !current_adc_channel;
    adc_select_input(current_adc_channel);
}

static int64_t adc_timer_callback(__unused alarm_id_t id, __unused void *user_data) {
    adc_values[current_adc_channel] = adc_read() * ADC_CONVERT;
    current_adc_channel = !current_adc_channel;
    adc_select_input(current_adc_channel);

    return ADC_TIMER_TIME;
}

void analog_io_init() { 
    adc_init();
    adc_gpio_init(ADC_PIN_PORT_MEAS);
    adc_gpio_init(ADC_PIN_PORT_STBD);

    current_adc_channel = 0;
    adc_select_input(current_adc_channel);

    hard_assert(add_alarm_in_ms(ADC_TIMER_TIME, adc_timer_callback, NULL, true) > 0);
}

uint16_t analog_io_read_port_meas() { 
    return adc_values[ADC_NUM_PORT_MEAS];
}

uint16_t analog_io_read_stbd_meas() { 
    return adc_values[ADC_NUM_STBD_MEAS];
}