#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "basic_logging/logging.h"

#include "drivers/adc.h"
#include "drivers/safety.h"
#include "hw/esc_adc.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "esc_adc"

#define max(num1, num2) (num1 > num2 ? num1 : num2)

static void esc_adc_failure(__unused const struct async_i2c_request * req, uint32_t abort_data) {
    LOG_ERROR("Failed to read ESC adc (Abort Data: %d)", abort_data);
    safety_raise_fault(FAULT_ESC_ADC_ERROR);
}

const struct adc_configuration esc_adc_config = {
    .i2c = BOARD_I2C_HW,
    .address = 0x2F,
    .poll_rate_ms = 250,
    .enable_temperature = false,
    .external_vref = true,
    .monitored_channels = 0b11111111,
    .failed_init_callback = esc_adc_failure,
    .failed_read_callback = esc_adc_failure
};

struct adc_instance esc_adc_inst = {0};

double esc_adc_get_thruster_current(int thruster_id) {
    double voltage = adc_get_reading(&esc_adc_inst, thruster_id) * 3.3 / 4096;
	return max((voltage - .33) / .264, 0);
}

void esc_adc_init(void) {
    adc_start(&esc_adc_inst, &esc_adc_config);
}