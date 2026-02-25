#ifndef FFT_H
#define FFT_H

#include "fft/kiss_fftr.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

#include <math.h>
#include <stdio.h>

#define CLOCK_DIV 0
#define FSAMP 500000
#define CAPTURE_CHANNEL 1
#define NSAMP 500

typedef struct {
    const char *name;
    int freq_min;
    int freq_max;
    float amplitude;
} frequency_bin_t;

void fft_setup(irq_handler_t dma_irq_cb);
void fft_sample(uint8_t *capture_buf);
void fft_process(uint8_t *capture_buf, frequency_bin_t *bins, int bin_count);

#endif /* FFT_H */
