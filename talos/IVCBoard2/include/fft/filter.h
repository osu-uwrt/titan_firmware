#ifndef FILTER_H
#define FILTER_H

#include "fft/fft.h"

#include <stdint.h>

float fir_filter(uint8_t adc_sample);
int32_t fir_filter_int(uint8_t adc_sample);
int32_t fir_filter_pinger(uint8_t adc_sample);
int32_t fir_filter_pinger_20k(uint8_t adc_sample);
int32_t fir_filter_pinger_25k(uint8_t adc_sample);
int32_t fir_filter_pinger_30k(uint8_t adc_sample);
int32_t fir_filter_pinger_35k(uint8_t adc_sample);
int32_t fir_filter_pinger_37k(uint8_t adc_sample);
int32_t fir_filter_pinger_40k(uint8_t adc_sample);
int32_t fir_filter_pinger_int(uint8_t adc_sample, pinger_freq_t freq);

#endif  // FILTER_H
