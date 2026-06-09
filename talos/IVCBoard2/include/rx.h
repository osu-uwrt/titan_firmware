#ifndef RX_H
#define RX_H

#include "ivc.h"

#include <stdint.h>

// #define FFT_LOW_IDX 0
// #define FFT_HIGH_IDX 1
// #define FFT_SYNC_IDX 2

// #define FFT_LOW_IDX 3
// #define FFT_HIGH_IDX 4
// #define FFT_SYNC_IDX 2

#define FFT_LOW_IDX 2
#define FFT_HIGH_IDX 3
#define FFT_SYNC_IDX 4
#define FFT_REF1_IDX 0
#define FFT_REF2_IDX 1
#define FFT_REF3_IDX 5
#define FFT_REF4_IDX 6

// scale floats to chop off decimal places with an integer cast ()
#define FLOAT_SCALE_MULTIPLIER 1000.0f

/**
 * @brief will attempt to read an incoming packet
 *
 * @param ctx pointer to the main context struct
 */
void attempt_reading(ivc_context_t *ctx);

/**
 * @brief gets a single symbol sample
 *
 * @param ctx pointer to the main context struct
 *
 * @return sample_t the sample observed at the point in time of calling this function
 */
sample_t rx_observe(ivc_context_t *ctx);

/**
 * @brief initializes rx hardware and fft logic
 *
 * @param ctx pointer to the main context struct
 * @param swap_buffer_handler interupt that fires after dma finishes filling a swap buffer with adc samples
 */
void rx_init(ivc_context_t *ctx, void (*swap_buffer_handler)());

float rx_observe_amplitude(ivc_context_t *ctx);
float get_low_amp();
float get_sync_amp();
float get_high_amp();
float get_ref1_amp();
float get_ref2_amp();
float get_ref3_amp();
float get_ref4_amp();
sample_t rx_observe_3(ivc_context_t *ctx);
void process_fft(ivc_context_t *ctx);

#endif  // RX_H
