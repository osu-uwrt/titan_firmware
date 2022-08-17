#ifndef _BALANCER_ADC_H
#define _BALANCER_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/adc.h"

/**
 * @brief How long until the reading from the ADC should be considered stale.
 * This should be calculated based on the poll_ms in the adc config struct
 */
#define BALANCER_ADC_STALE_READING_AGE_MS 1000

extern struct adc_instance balancer_adc_inst;

#define balancer_adc_initialized (balancer_adc_inst.initialized)
#define balancer_adc_last_reading (balancer_adc_inst.last_reading_time)
#define balancer_adc_readng_stale (absolute_time_diff_us(delayed_by_ms(balancer_adc_last_reading, BALANCER_ADC_STALE_READING_AGE_MS), get_absolute_time()) > 0)

double balancer_adc_get_stbd_current(void);
double balancer_adc_get_port_current(void);
double balancer_adc_get_balanced_voltage(void);
double balancer_adc_get_stbd_voltage(void);
double balancer_adc_get_port_voltage(void);
double balancer_adc_get_temperature(void);
void balancer_adc_init(void);

#endif