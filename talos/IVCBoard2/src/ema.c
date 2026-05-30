
#include "fft/fft.h"
#include "ivc.h"

// τ should settle in ~15% of symbol = 30ms = 30 frames
// α = 1 / τ_frames = 1/30 ≈ 0.05
#define SIGNAL_ALPHA 0.05f   // τ ≈ 20 frames = 20ms
#define NOISE_ALPHA 0.0005f  // τ ≈ 2000 frames = 2 seconds -

// typedef struct {
//     float signal[NUM_FFT_BINS];  // less resistant to noise
//     float noise[NUM_FFT_BINS];   // more resistant to noise
//     bool initialized;
// } amplitude_ema_t;

float ema_snr(amplitude_ema_t *ema, uint8_t bin_idx) {
    return ema->signal[bin_idx] / (ema->noise[bin_idx] + 1e-6f);
}

void ema_update(amplitude_ema_t *ema, frequency_bin_t bins[]) {
    if (!ema->initialized) {
        for (uint8_t i = 0; i < NUM_FFT_BINS; i++) {
            ema->signal[i] = ema->noise[i] = bins[i].amplitude;
        }
        ema->initialized = true;
        return;
    }

    for (uint8_t i = 0; i < NUM_FFT_BINS; i++) {
        float amp = bins[i].amplitude;

        ema->signal[i] = SIGNAL_ALPHA * amp + (1.0f - SIGNAL_ALPHA) * ema->signal[i];

        // update noise when things look quiet
        if (amp < ema->noise[i] * 3.0f) {
            ema->noise[i] = NOISE_ALPHA * amp + (1.0f - NOISE_ALPHA) * ema->noise[i];
        }
    }
}
