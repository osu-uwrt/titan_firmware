#include "fft/filter.h"

#include "fft/fft.h"

#define N_FIR_TAPS 64
#define FIR_SCALE_SHIFT 15  // 2^15 = 32768

/**
 * if the float math here isn't done in integers, nothing will happen and the rp2040 will fucking die because its
 * flops are done in SOFTWARE
 *
 * and since this shit is happening every sample, we need all the help we can get
 *
 * doing the filter with ints rather than floats leads to a maximum difference per sample of
 * Max difference: 3.853616440530061e-0 (negligable)
 * according to claudes python script
 */

// generated from python lib, firwin w/ raw nsamp = 4096, raw fsamp = 500khz, bound = (48.5 khz, 53.5 khz)
static const float fir_coeffs[N_FIR_TAPS] = {
    0.0000000000f,  0.0001054668f,  0.0005539316f,  0.0010474445f,  0.0007592133f,  -0.0010211100f, -0.0040678170f,
    -0.0068596106f, -0.0071170246f, -0.0031297881f, 0.0048136421f,  0.0138256467f,  0.0192719404f,  0.0169396203f,
    0.0055557122f,  -0.0117200207f, -0.0278916521f, -0.0349074484f, -0.0275402550f, -0.0065181411f, 0.0208284048f,
    0.0430916603f,  0.0496435915f,  0.0357132957f,  0.0052402195f,  -0.0300845960f, -0.0554631408f, -0.0594142141f,
    -0.0390960901f, -0.0020036895f, 0.0369059934f,  0.0615160068f,  0.0615160068f,  0.0369059934f,  -0.0020036895f,
    -0.0390960901f, -0.0594142141f, -0.0554631408f, -0.0300845960f, 0.0052402195f,  0.0357132957f,  0.0496435915f,
    0.0430916603f,  0.0208284048f,  -0.0065181411f, -0.0275402550f, -0.0349074484f, -0.0278916521f, -0.0117200207f,
    0.0055557122f,  0.0169396203f,  0.0192719404f,  0.0138256467f,  0.0048136421f,  -0.0031297881f, -0.0071170246f,
    -0.0068596106f, -0.0040678170f, -0.0010211100f, 0.0007592133f,  0.0010474445f,  0.0005539316f,  0.0001054668f,
    0.0000000000f,
};

static const int16_t fir_coeffs_int[64] = {
    0,    3,     18,   34,    25,    -33,   -133, -225, -233, -103, 158,   453,   632,   555,  182,   -384,
    -914, -1144, -902, -214,  683,   1412,  1627, 1170, 172,  -986, -1817, -1947, -1281, -66,  1209,  2016,
    2016, 1209,  -66,  -1281, -1947, -1817, -986, 172,  1170, 1627, 1412,  683,   -214,  -902, -1144, -914,
    -384, 182,   555,  632,   453,   158,   -103, -233, -225, -133, -33,   25,    34,    18,   3,     0,
};

static float fir_history[N_FIR_TAPS] = { 0 };
static uint32_t fir_history_idx = 0;

static int16_t fir_history_int[N_FIR_TAPS] = { 0 };

int32_t fir_filter_int(uint8_t adc_sample) {
    fir_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) fir_coeffs_int[i] * (int32_t) fir_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

float fir_filter(uint8_t adc_sample) {
    fir_history[fir_history_idx] = (float) adc_sample;

    float sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += fir_coeffs[i] * fir_history[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum;
}
