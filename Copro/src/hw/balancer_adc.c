#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "basic_logger/logging.h"

#include "drivers/adc.h"
#include "drivers/safety.h"
#include "hw/balancer_adc.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "balancer_adc"

#define max(num1, num2) (num1 > num2 ? num1 : num2)

static void balancer_adc_failure(__unused const struct async_i2c_request * req, uint32_t abort_data) {
    LOG_ERROR("Failed to read BB adc (Abort Data: %d)", abort_data);
    safety_raise_fault(FAULT_BB_ADC_ERROR);
}

const struct adc_configuration balancer_adc_config = {
    .i2c_num = SENSOR_I2C,
    .address = 0x1F,
    .poll_rate_ms = 250,
    .enable_temperature = true,
    .external_vref = true,
    .monitored_channels = 0b10011111,
    .failed_init_callback = balancer_adc_failure,
    .failed_read_callback = balancer_adc_failure
};

struct adc_instance balancer_adc_inst = {0};

double balancer_adc_get_stbd_current(void) {
    double voltage = adc_get_reading(&balancer_adc_inst, 0) * 3.3 / 4096;
    return max((voltage - .33) / .066, 0);
}

double balancer_adc_get_port_current(void) {
    double voltage = adc_get_reading(&balancer_adc_inst, 1) * 3.3 / 4096;
    return max((voltage - .33) / .066, 0);
}

double balancer_adc_get_balanced_voltage(void) {
    return adc_get_reading(&balancer_adc_inst, 2) * 3.3 / 4096 * (118.0 / 18) * .958;
}

double balancer_adc_get_stbd_voltage(void) {
    return adc_get_reading(&balancer_adc_inst, 3) * 3.3 / 4096 * (118.0 / 18)* .958;
}

double balancer_adc_get_port_voltage(void) {
    return adc_get_reading(&balancer_adc_inst, 4) * 3.3 / 4096 * (118.0 / 18)* .958;
}

double balancer_adc_get_temperature(void) {
    return adc_get_reading(&balancer_adc_inst, 7)/16.0;
}

void balancer_adc_init(void) {
    adc_start(&balancer_adc_inst, &balancer_adc_config);
}