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
#define NUM_FFT_BINS 7
#define NUM_ACCUMULATE 1

#define FSAMP 500000  // ADC sample rate
#define DECIMATE_BY 4
#define FSAMP_DEC (FSAMP / DECIMATE_BY)
#define NSAMP 4096  // raw ADC capture size
#define NSAMP_DEC (NSAMP / DECIMATE_BY)
#define KISS_FFT_ALLOC_SIZE 10752 // bytes

typedef struct {
    const char *name;
    int freq_min;
    int freq_max;
    float amplitude;
    bool is_reference_bin;
} frequency_bin_t;

void fft_setup(irq_handler_t dma_irq_cb);
void fft_sample(uint8_t *capture_buf);
void fft_process(uint8_t *capture_buf, frequency_bin_t *bins, int bin_count);
bool fft_get_noise_floor(float *noise_floor);
void precompute_hann_window();

#endif /* FFT_H */
