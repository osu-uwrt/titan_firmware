#include "fft/fft.h"

#include "fft/filter.h"

#include "titan/logger.h"

#include <math.h>

static dma_channel_config cfg;
static uint dma_chan;
static float freqs[NSAMP];
static irq_handler_t dma_cb;

// precomputed at init
static float hann_window[NSAMP_DEC] = { 0 };
static float frequencies[NSAMP_DEC] = { 0 };
static float accum_window[NUM_FFT_BINS] = { 0 };
static uint32_t accum_counter = 0;
static uint8_t kiss_mem[KISS_FFT_ALLOC_SIZE];
static kiss_fftr_cfg fft_cfg;
static size_t kiss_mem_len = KISS_FFT_ALLOC_SIZE;

#define EMA_ALPHA 0.05f
static float noise_floor_ema = 0.0f;
static uint32_t num_frames_processed = 0;
static bool noise_floor_settled = false;

static void calculate_frequencies();
static float calculate_average(uint8_t *buffer, int size);
static void fill_fft_input(uint8_t *buffer, kiss_fft_scalar *fft_in, int size);
static void reset_bins(frequency_bin_t *bins, int bin_count);
static void compute_bin_amplitudes(kiss_fft_cpx *fft_out, frequency_bin_t *bins, int bin_count, int nsamp);
static void dma_handler();

static void calculate_frequencies_dec();

void fft_setup(irq_handler_t dma_irq_cb) {
    stdio_init_all();
    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(true,   // Write each completed conversion to the sample FIFO
                   true,   // Enable DMA data request (DREQ)
                   1,      // DREQ (and IRQ) asserted when at least 1 sample present
                   false,  // We won't see the ERR bit because of 8 bit reads; disable.
                   true    // Shift each sample to 8 bits when pushing to FIFO
    );

    adc_set_clkdiv(CLOCK_DIV);
    sleep_ms(1000);

    dma_chan = dma_claim_unused_channel(true);
    if (dma_chan == -1) {
        fprintf(stderr, "Failed to claim unused DMA channel\n");
        return;
    }

    cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_cb = dma_irq_cb;
    dma_channel_set_irq0_enabled(dma_chan, true);  // Set IRQ
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // calculate_frequencies();
    calculate_frequencies_dec();
    precompute_hann_window();
    fft_cfg = kiss_fftr_alloc(NSAMP_DEC, false, &kiss_mem, &kiss_mem_len);
    if (!fft_cfg) {
        fprintf(stderr, "Failed to allocate FFT configuration\n");
        return;
    }
}

// new

static void process_capture_buffer(uint8_t capture_buf[], kiss_fft_scalar fft_in[]) {
    uint8_t decimation_counter = 0;
    uint32_t decimated_idx = 0;

    for (uint32_t i = 0; i < NSAMP; i++) {
        float filtered_sample = (float) fir_filter_int(capture_buf[i]);
        // float filtered_sample = 4;
        if (decimation_counter == 0) {
            fft_in[decimated_idx++] = filtered_sample;
        }
        decimation_counter = (decimation_counter + 1) % DECIMATE_BY;
    }

    // do hann on decimated_buffer
}

static void remove_dc_offset(kiss_fft_scalar fft_in[]) {
    float sum = 0;
    for (uint32_t i = 0; i < NSAMP_DEC; i++) {
        sum += fft_in[i];
    }

    float avg = sum / NSAMP_DEC;
    for (uint32_t i = 0; i < NSAMP_DEC; i++) {
        fft_in[i] -= avg;
    }
}

static void apply_hann_window(kiss_fft_scalar fft_in[]) {
    for (uint32_t i = 0; i < NSAMP_DEC; i++) {
        fft_in[i] *= hann_window[i];
    }
}

void precompute_hann_window() {
    for (uint32_t i = 0; i < NSAMP_DEC; i++) {
        hann_window[i] = 0.5f * (1.0f - cosf(2.0 * M_PI * i / (NSAMP_DEC - 1)));
    }
}

static void calculate_frequencies_dec() {
    float f_max = FSAMP_DEC;
    float f_res = f_max / NSAMP_DEC;
    for (int i = 0; i < NSAMP_DEC; i++) {
        frequencies[i] = f_res * i;
    }
}

// static void compute_bin_amplitudes_dec(kiss_fft_cpx *fft_out, frequency_bin_t *bins, uint32_t num_bins) {
//     for (int i = 0; i < NSAMP_DEC / 2; i++) {
//         float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
//         float freq = frequencies[i];

//         for (int j = 0; j < num_bins; j++) {
//             if (freq >= bins[j].freq_min && freq <= bins[j].freq_max) {
//                 bins[j].amplitude += power;
//                 break;
//             }
//         }
//     }

//     for (int i = 0; i < num_bins; i++) {
//         bins[i].amplitude = sqrtf(bins[i].amplitude);
//     }
// }

static void accumulate_fft_window(kiss_fft_cpx fft_out[], frequency_bin_t bins[], uint32_t num_bins) {
    for (int i = 0; i < NSAMP_DEC / 2; i++) {
        float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
        float freq = frequencies[i];

        for (int j = 0; j < num_bins; j++) {
            if (freq >= bins[j].freq_min && freq <= bins[j].freq_max) {
                accum_window[j] += power;
                break;
            }
        }
    }
    accum_counter++;
}

static void compute_accumulated_amplitudes(frequency_bin_t bins[], uint32_t num_bins) {
    size_t num_ref_bins = 0;
    float ref_bin_mean = 0.0f;
    for (int i = 0; i < num_bins; i++) {
        bins[i].amplitude = sqrtf(accum_window[i] / NUM_ACCUMULATE);
        if (bins[i].is_reference_bin) {
            ref_bin_mean += bins[i].amplitude;
            num_ref_bins++;
        }
    }
    if (num_ref_bins > 0) {
        ref_bin_mean /= num_ref_bins;
        noise_floor_ema = EMA_ALPHA * ref_bin_mean + (1.0f - EMA_ALPHA) * noise_floor_ema;
    }
    // LOG_INFO("EMA noise floor: %0.5f\n", noise_floor_ema);
    // for (int i = 0; i < num_bins; i++) {
    // LOG_INFO("bin[%d] min=%f max=%f amp=%f", i, bins[i].freq_min, bins[i].freq_max, bins[i].amplitude);
    // }
}

static void reset_accum_window() {
    for (int i = 0; i < NUM_FFT_BINS; i++) {
        accum_window[i] = 0.0f;
    }
}

bool fft_get_noise_floor(float *noise_floor) {
    if (noise_floor_settled) {
        *noise_floor = noise_floor_ema;
        return true;
    }
    else {
        return false;
    }

    // if (noise_floor_ema < 1e-6f) return 1e-6f;
    // return noise_floor_ema;
}

// end new

void fft_sample(uint8_t *capture_buf) {
    // adc_run(false);
    // adc_fifo_drain();

    // LOG_INFO("Starting DMA");

    dma_channel_configure(dma_chan, &cfg,
                          capture_buf,    // dst
                          &adc_hw->fifo,  // src
                          NSAMP,          // transfer count
                          true            // start immediately
    );

    adc_run(true);
    // dma_channel_wait_for_finish_blocking(dma_chan);
}

void check_bins(frequency_bin_t bins[]) {
    for (int i = 0; i < NUM_FFT_BINS; i++) {
        LOG_INFO("bin[%d] %s min=%f max=%f ref=%d", i, bins[i].name, bins[i].freq_min, bins[i].freq_max,
                 bins[i].is_reference_bin);
    }
}

// void fft_process(uint8_t *capture_buf, frequency_bin_t *bins, int bin_count) {
//     kiss_fft_scalar fft_in[NSAMP];
//     kiss_fft_cpx fft_out[NSAMP];
//     kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, NULL, NULL);

//     if (!cfg) {
//         fprintf(stderr, "Failed to allocate FFT configuration\n");
//         return;
//     }

//     fill_fft_input(capture_buf, fft_in, NSAMP);
//     kiss_fftr(cfg, fft_in, fft_out);
//     reset_bins(bins, bin_count);
//     compute_bin_amplitudes(fft_out, bins, bin_count, NSAMP);
//     kiss_fft_free(cfg);
// }

// just return ema from here
void fft_process(uint8_t capture_buf[], frequency_bin_t bins[], int bin_count) {
    kiss_fft_scalar fft_in[NSAMP_DEC];
    kiss_fft_cpx fft_out[NSAMP_DEC];
    // size_t needed;
    //  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP_DEC, false, NULL, &needed);

    // if (!cfg) {
    //     fprintf(stderr, "Failed to allocate FFT configuration\n");
    //     return;
    // }
    process_capture_buffer(capture_buf, fft_in);
    remove_dc_offset(fft_in);
    apply_hann_window(fft_in);
    kiss_fftr(fft_cfg, fft_in, fft_out);
    // LOG_INFO("\nbin414=%f\nbin418=%f\nbin422=%f\n", fft_out[414].r * fft_out[414].r + fft_out[414].i *
    // fft_out[414].i,
    //          fft_out[418].r * fft_out[418].r + fft_out[418].i * fft_out[418].i,
    //          fft_out[422].r * fft_out[422].r + fft_out[422].i * fft_out[422].i);

    accumulate_fft_window(fft_out, bins, bin_count);
    if (accum_counter == NUM_ACCUMULATE) {
        reset_bins(bins, bin_count);
        compute_accumulated_amplitudes(bins, bin_count);
        reset_accum_window();
        accum_counter = 0;
        num_frames_processed++;
        if (num_frames_processed >= 20) {
            noise_floor_settled = true;
        }
    }
    // check_bins(bins);
    //  reset_bins(bins, bin_count);
    //  compute_bin_amplitudes_dec(fft_out, bins, bin_count);
    // kiss_fft_free(cfg);
}

static void dma_handler() {
    // LOG_INFO("Got DMA IRQ");

    dma_cb();

    // Clear the interrupt request
    dma_hw->ints0 = 1u << dma_chan;
}

static void calculate_frequencies() {
    float f_max = FSAMP;
    float f_res = f_max / NSAMP;
    for (int i = 0; i < NSAMP; i++) {
        freqs[i] = f_res * i;
    }
}

static float calculate_average(uint8_t *buffer, int size) {
    uint64_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return (float) sum / size;
}

static void fill_fft_input(uint8_t *buffer, kiss_fft_scalar *fft_in, int size) {
    float avg = calculate_average(buffer, size);
    for (int i = 0; i < size; i++) {
        fft_in[i] = (float) buffer[i] - avg;
    }
}

static void reset_bins(frequency_bin_t *bins, int bin_count) {
    for (int i = 0; i < bin_count; i++) {
        bins[i].amplitude = 0.0f;
    }
}

static void compute_bin_amplitudes(kiss_fft_cpx *fft_out, frequency_bin_t *bins, int bin_count, int nsamp) {
    for (int i = 0; i < nsamp / 2; i++) {
        float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
        float freq = freqs[i];

        for (int j = 0; j < bin_count; j++) {
            if (freq >= bins[j].freq_min && freq <= bins[j].freq_max) {
                bins[j].amplitude += power;
                break;
            }
        }
    }

    for (int i = 0; i < bin_count; i++) {
        bins[i].amplitude = sqrtf(bins[i].amplitude);
    }
}
