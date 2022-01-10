#ifndef _ESC_ADC_H
#define _ESC_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/adc.h"

extern struct adc_instance esc_adc_inst;

#define esc_adc_initialized (esc_adc_inst->initialized);
double esc_adc_get_thruster_current(int thruster_id);
void esc_adc_init(void);

#endif