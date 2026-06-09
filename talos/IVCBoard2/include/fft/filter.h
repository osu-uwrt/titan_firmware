#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

float fir_filter(uint8_t adc_sample);
int32_t fir_filter_int(uint8_t adc_sample);

#endif  // FILTER_H
