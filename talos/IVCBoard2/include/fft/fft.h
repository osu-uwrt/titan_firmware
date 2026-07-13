#ifndef FFT_H
#define FFT_H

#include "fft/kiss_fftr.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

#include <math.h>
#include <stdio.h>
// #define CLOCK_DIV 1000
#define CLOCK_DIV 0
#define FSAMP 500000
// #define FSAMP 48000
#define CAPTURE_CHANNEL 1
// last test
// #define NSAMP 1000

// #define NSAMP 100
// #define NSAMP 256
#define NUM_FFT_BINS 8
#define NUM_PINGER_FFT_BINS 5
#define NUM_PINGER_FREQS 6
#define NUM_ACCUMULATE 1

#define FSAMP 500000  // ADC sample rate
#define DECIMATE_BY 4
#define FSAMP_DEC (FSAMP / DECIMATE_BY)
#define NSAMP 4096  // raw ADC capture size
#define NSAMP_DEC (NSAMP / DECIMATE_BY)
#define KISS_FFT_ALLOC_SIZE 10752  // bytes

typedef int32_t (*filter_func)(uint8_t adc_sample);

typedef struct {
    const char *name;
    int freq_min;
    int freq_max;
    float amplitude;
    bool is_reference_bin;  // for noise ema calc
} frequency_bin_t;

typedef struct {
    frequency_bin_t *bins;
    uint8_t bins_count;
} fft_config_t;

typedef enum {
    FREQ_20KHZ,
    FREQ_25KHZ,
    FREQ_30KHZ,
    FREQ_37KHZ,
    FREQ_35KHZ,
    FREQ_40KHZ,
} pinger_freq_t;

void fft_setup(irq_handler_t dma_irq_cb, bool *is_pinger, pinger_freq_t *mode);
void fft_sample(uint8_t *capture_buf);
void fft_process(uint8_t *capture_buf, frequency_bin_t *bins, int bin_count);
bool fft_get_noise_floor(float *noise_floor);
void precompute_hann_window();

#endif /* FFT_H */
