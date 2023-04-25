#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "analog_io.h"
#include "hardware/adc.h"

#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

#define PORT_MEAS_ADC_CHAN (PORT_MEAS_PIN - 26)
#define STBD_MEAS_ADC_CHAN (STBD_MEAS_PIN - 26)
static_assert(0 <= PORT_MEAS_ADC_CHAN && PORT_MEAS_ADC_CHAN <= 3, "Invalid port measurement pin");
static_assert(0 <= STBD_MEAS_ADC_CHAN && STBD_MEAS_ADC_CHAN <= 3, "Invalid starboard measurement pin");

#define ADC_TIMER_TIME_MS 100

static float port_measurement;
static float stbd_measurement;
static int current_adc_channel;

static int64_t adc_timer_callback(__unused alarm_id_t id, __unused void *user_data) {
    // Note that this technically blocks for 2us, but honestly doesn't matter at 100ms polling rate
    uint16_t adc_data = adc_read();
    float voltage = ((float) adc_data) * ADC_CONVERT;
    float top_resistor = 10000.0;
    float bottom_bottom = 1000.0;

    voltage *= 1.0 / (bottom_bottom / (bottom_bottom + top_resistor));
    if (current_adc_channel == STBD_MEAS_ADC_CHAN) {
        stbd_measurement = voltage;
        current_adc_channel = PORT_MEAS_ADC_CHAN;
    }
    else {
        port_measurement = voltage;
        current_adc_channel = STBD_MEAS_ADC_CHAN;
    }
    adc_select_input(current_adc_channel);

    return ADC_TIMER_TIME_MS * 1000;
}

void analog_io_init() {
    adc_init();
    bi_decl_if_func_used(bi_1pin_with_name(PORT_MEAS_PIN, "Port Voltage Measurement"));
    bi_decl_if_func_used(bi_1pin_with_name(STBD_MEAS_PIN, "Starboard Voltage Measurement"));
    adc_gpio_init(PORT_MEAS_PIN);
    adc_gpio_init(STBD_MEAS_PIN);

    current_adc_channel = STBD_MEAS_ADC_CHAN;
    adc_select_input(current_adc_channel);

    hard_assert(add_alarm_in_ms(ADC_TIMER_TIME_MS, adc_timer_callback, NULL, true) > 0);
}

float analog_io_read_port_meas() {
    return port_measurement;
}

float analog_io_read_stbd_meas() {
    return stbd_measurement;
}