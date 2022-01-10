#ifndef _BALANCER_ADC_H
#define _BALANCER_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/adc.h"

extern struct adc_instance balancer_adc_inst;

#define balancer_adc_initialized (balancer_adc_inst->initialized);
double balancer_adc_get_stbd_current(void);
double balancer_adc_get_port_current(void);
double balancer_adc_get_balanced_voltage(void);
double balancer_adc_get_stbd_voltage(void);
double balancer_adc_get_port_voltage(void);
double balancer_adc_get_temperature(void);
void balancer_adc_init(void);

#endif