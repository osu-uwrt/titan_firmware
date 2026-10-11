#include "fft/filter.h"

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

static const int16_t pinger_coeffs_int[64] = {
    // 37khzs
    0,    0,    8,    33,   75,   116,   130,   89,    -21,   -191,  -386,  -545, -602, -504, -235, 172,
    635,  1039, 1264, 1218, 870,  264,   -481,  -1198, -1709, -1873, -1624, -996, -115, 824,  1607, 2050,
    2050, 1607, 824,  -115, -996, -1624, -1873, -1709, -1198, -481,  264,   870,  1218, 1264, 1039, 635,
    172,  -235, -504, -602, -545, -386,  -191,  -21,   89,    130,   116,   75,   33,   8,    0,    0,

};

static const int16_t pinger_coeffs_20k_int[64] = {
    0,    1,     8,     27,    62,    111,   170,   230,  279,  304,   293,   233,   120,   -47,   -261,  -508,
    -768, -1016, -1225, -1367, -1421, -1370, -1206, -932, -561, -118,  365,   852,   1302,  1678,  1949,  2091,
    2091, 1949,  1678,  1302,  852,   365,   -118,  -561, -932, -1206, -1370, -1421, -1367, -1225, -1016, -768,
    -508, -261,  -47,   120,   233,   293,   304,   279,  230,  170,   111,   62,    27,    8,     1,     0,
};

static const int16_t pinger_coeffs_25k_int[64] = {
    0,    -5,   -19,  -38,  -54,   -54,   -27,   36,    136,   266,   410,   542,   635,  661,  597,  431,
    165,  -182, -575, -968, -1309, -1546, -1635, -1551, -1286, -857,  -305,  313,   928,  1469, 1870, 2084,
    2084, 1870, 1469, 928,  313,   -305,  -857,  -1286, -1551, -1635, -1546, -1309, -968, -575, -182, 165,
    431,  597,  661,  635,  542,   410,   266,   136,   36,    -27,   -54,   -54,   -38,  -19,  -5,   0,
};

static const int16_t pinger_coeffs_30k_int[64] = {
    0,    2,    2,    -11, -45,  -101,  -168,  -227,  -253,  -221,  -114,  69,   309, 571,  802,  945,
    953,  793,  465,  0,   -540, -1070, -1496, -1734, -1727, -1453, -938,  -251, 508, 1219, 1769, 2069,
    2069, 1769, 1219, 508, -251, -938,  -1453, -1727, -1734, -1496, -1070, -540, 0,   465,  793,  953,
    945,  802,  571,  309, 69,   -114,  -221,  -253,  -227,  -168,  -101,  -45,  -11, 2,    2,    0,
};

static const int16_t pinger_coeffs_35k_int[64] = {
    0,    3,    17,   43,   68,   73,    37,    -50,   -184,  -335,  -459,  -503, -425, -207, 132,  533,
    907,  1155, 1190, 967,  497,  -147,  -842,  -1439, -1795, -1812, -1462, -795, 64,   942,  1657, 2057,
    2057, 1657, 942,  64,   -795, -1462, -1812, -1795, -1439, -842,  -147,  497,  967,  1190, 1155, 907,
    533,  132,  -207, -425, -503, -459,  -335,  -184,  -50,   37,    73,    68,   43,   17,   3,    0,
};

static const int16_t pinger_coeffs_37k_int[64] = {
    // 37khzs
    0,    0,    8,    33,   75,   116,   130,   89,    -21,   -191,  -386,  -545, -602, -504, -235, 172,
    635,  1039, 1264, 1218, 870,  264,   -481,  -1198, -1709, -1873, -1624, -996, -115, 824,  1607, 2050,
    2050, 1607, 824,  -115, -996, -1624, -1873, -1709, -1198, -481,  264,   870,  1218, 1264, 1039, 635,
    172,  -235, -504, -602, -545, -386,  -191,  -21,   89,    130,   116,   75,   33,   8,    0,    0,

};

static const int16_t pinger_coeffs_40k_int[64] = {
    0,    -4,   -12,  -8,   24,    87,    166,   224,   219,   116,   -86,   -350,  -598, -736, -683, -404,
    66,   621,  1108, 1368, 1286,  838,   104,   -740,  -1470, -1872, -1811, -1275, -383, 641,  1529, 2042,
    2042, 1529, 641,  -383, -1275, -1811, -1872, -1470, -740,  104,   838,   1286,  1368, 1108, 621,  66,
    -404, -683, -736, -598, -350,  -86,   116,   219,   224,   166,   87,    24,    -8,   -12,  -4,   0,
};

static int16_t *pinger_coeffs_map[] = {
    [FREQ_20KHZ] = pinger_coeffs_20k_int, [FREQ_25KHZ] = pinger_coeffs_25k_int, [FREQ_30KHZ] = pinger_coeffs_30k_int,
    [FREQ_35KHZ] = pinger_coeffs_35k_int, [FREQ_37KHZ] = pinger_coeffs_37k_int, [FREQ_40KHZ] = pinger_coeffs_40k_int,
};

static float fir_history[N_FIR_TAPS] = { 0 };
static uint32_t fir_history_idx = 0;

static int16_t fir_history_int[N_FIR_TAPS] = { 0 };
static int16_t pinger_history_int[N_FIR_TAPS] = { 0 };

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

int32_t fir_filter_pinger_int(uint8_t adc_sample, pinger_freq_t freq) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_map[freq][i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_20k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_20k_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_25k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_25k_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_30k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_30k_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_35k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_35k_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_37k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}

int32_t fir_filter_pinger_40k(uint8_t adc_sample) {
    pinger_history_int[fir_history_idx] = (int16_t) adc_sample;

    int32_t sum = 0;
    for (uint32_t i = 0; i < N_FIR_TAPS; i++) {
        sum += (int32_t) pinger_coeffs_40k_int[i] *
               (int32_t) pinger_history_int[(fir_history_idx + N_FIR_TAPS - i) % N_FIR_TAPS];
    }

    fir_history_idx = (fir_history_idx + 1) % N_FIR_TAPS;
    return sum >> FIR_SCALE_SHIFT;
}
