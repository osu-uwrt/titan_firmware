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

#define FFT_LOW_IDX 3
#define FFT_HIGH_IDX 4
#define FFT_SYNC_IDX 5
#define FFT_REF1_IDX 1
#define FFT_REF2_IDX 2
#define FFT_REF3_IDX 6
#define FFT_REF4_IDX 7
#define FFT_PINGER_IDX 0

#define FFT_PINGER_SIGNAL_IDX 2

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
float get_pinger_amp();
sample_t rx_observe_3(ivc_context_t *ctx);
void process_fft(ivc_context_t *ctx);
void rx_timeout_start();
void rx_timeout_advance();
bool rx_timeout_expired();
void rx_timeout(ivc_context_t *ctx);
bool rx_observe_pinger(ivc_context_t *ctx, float *avg);
bool push_pinger_amp(float amp);
void rx_set_pinger_bins(ivc_context_t *ctx, pinger_freq_t freq);
void rx_handle_pinger(ivc_context_t *ctx);
int32_t rx_get_pinger_mode_khz(ivc_context_t *ctx);

#endif  // RX_H
