#include "hardware/adc.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define ADC_THRESHOLD 3500  // Arbitrary threshold to identify peak (can be adjusted)

uint32_t last_peak_time_adc0 = 0;
uint32_t last_peak_time_adc1 = 0;
uint16_t last_value_adc0 = 0;
uint16_t last_value_adc1 = 0;

int main() {
    // Initialize the standard I/O (USB Serial) for output
    stdio_init_all();

    // Initialize ADC (ADC0 and ADC1 for GPIO26 and GPIO27)
    adc_init();

    // Set the ADC input to GPIO26 (ADC0) and GPIO27 (ADC1)
    adc_gpio_init(26);  // ADC0 (GPIO26)
    adc_gpio_init(27);  // ADC1 (GPIO27)

    // Set the ADC width to 12 bits (default)
    adc_set_temp_sensor_enabled(true);

    uint32_t start_time = 0;

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Read the ADC value from GPIO26 (ADC0)
        adc_select_input(26);                  // Select ADC0 (GPIO26)
        uint16_t raw_value_adc0 = adc_read();  // Read the analog value

        // Check if ADC0 reaches its peak
        if (raw_value_adc0 > last_value_adc0 && raw_value_adc0 > ADC_THRESHOLD) {
            last_value_adc0 = raw_value_adc0;
            last_peak_time_adc0 = current_time;
            printf("ADC0 Peak Value: %u, Time: %u ms\n", raw_value_adc0, last_peak_time_adc0);
        }

        // Read the ADC value from GPIO27 (ADC1)
        adc_select_input(27);                  // Select ADC1 (GPIO27)
        uint16_t raw_value_adc1 = adc_read();  // Read the analog value

        // Check if ADC1 reaches its peak
        if (raw_value_adc1 > last_value_adc1 && raw_value_adc1 > ADC_THRESHOLD) {
            last_value_adc1 = raw_value_adc1;
            last_peak_time_adc1 = current_time;
            printf("ADC1 Peak Value: %u, Time: %u ms\n", raw_value_adc1, last_peak_time_adc1);
        }

        // Calculate time differential when both peaks are detected
        if (last_peak_time_adc0 != 0 && last_peak_time_adc1 != 0) {
            int32_t time_diff = (int32_t) (last_peak_time_adc1 - last_peak_time_adc0);
            printf("Time difference between peaks: %d ms\n", time_diff);
        }

        // Wait for a short time before the next reading
        sleep_ms(50);  // Adjust the delay for your application
    }

    return 0;
}
