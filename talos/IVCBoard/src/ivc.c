#include "ivc.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// General defines
#define FREQ_LOW_HZ 5000
#define FREQ_HIGH_HZ 10000
#define SYSCLK_HZ clock_get_hz(clk_sys)

// Tx data
// Define wrap such that clkdiv is on [0, 256)
#define PWM_WRAP_VALUE ((int) (((float) SYSCLK_HZ) / FREQ_LOW_HZ / 255.0f) + 1.0f)
static uint pwm_slice_num;
static int tx_data_idx;
static uint8_t tx_data;
static repeating_timer_t tx_timer = { 0 };

// Rx data

static bool tx_cb(__unused repeating_timer_t *rt) {
    if (tx_data_idx < 0) {
        pwm_set_enabled(pwm_slice_num, false);
        return false;
    }

    float freq = (tx_data >> tx_data_idx) & 0x01 ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
    pwm_set_clkdiv(pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));

    tx_data_idx--;
    return true;
}

void ivc_tx(uint8_t data) {
    tx_data_idx = 7;  // Transmit one byte's worth of data
    tx_data = data;

    pwm_set_enabled(pwm_slice_num, true);
    tx_cb(&tx_timer);
    add_repeating_timer_ms(-250, tx_cb, NULL, &tx_timer);
}

static void tx_init() {
    // Find out which PWM slice is connected to TX_PIN (it's slice 0)
    gpio_set_function(TX_PIN, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(TX_PIN);
    uint pwm_chan = pwm_gpio_to_channel(TX_PIN);

    pwm_set_wrap(pwm_slice_num, PWM_WRAP_VALUE - 1);
    pwm_set_chan_level(pwm_slice_num, pwm_chan, PWM_WRAP_VALUE / 2);

    // Select lpf output
    gpio_init(OUTPUT_SELECT_PIN);
    gpio_set_dir(OUTPUT_SELECT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_SELECT_PIN, 1);
}

static void rx_init() {}

void ivc_init() {
    tx_init();
    rx_init();
}
