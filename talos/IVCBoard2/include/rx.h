#ifndef RX_H
#define RX_H

#include "ivc.h"

#include <stdint.h>

#define FFT_LOW_IDX 0
#define FFT_HIGH_IDX 1
#define FFT_SYNC_IDX 2

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
void process_fft(ivc_context_t *ctx);

#endif  // RX_H
