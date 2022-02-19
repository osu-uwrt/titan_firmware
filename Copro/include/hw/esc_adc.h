#ifndef _ESC_ADC_H
#define _ESC_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/adc.h"

/**
 * @brief How long until the reading from the ADC should be considered stale.
 * This should be calculated based on the poll_ms in the adc config struct
 */
#define ESC_ADC_STALE_READING_AGE_MS 1000

extern struct adc_instance esc_adc_inst;

#define esc_adc_initialized (esc_adc_inst.initialized)
#define esc_adc_last_reading (esc_adc_inst.last_reading_time)
#define esc_adc_readng_stale (absolute_time_diff_us(delayed_by_ms(esc_adc_last_reading, ESC_ADC_STALE_READING_AGE_MS), get_absolute_time()) > 0)

double esc_adc_get_thruster_current(int thruster_id);
void esc_adc_init(void);

#endif