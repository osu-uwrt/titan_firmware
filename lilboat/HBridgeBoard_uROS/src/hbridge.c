#include "hbridge.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include <math.h>

#define PWM_FREQ_HZ 25000

uint ph;
uint slice_num;
uint channel;

static uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d) {
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);

    if (divider16 / 16 == 0)
        divider16 = 16;

    uint32_t wrap = clock * 16 / divider16 / f - 1;

    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);

    return wrap;
}

static void gpio_out_init(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

void hbridge_set_power(float power) {
    gpio_put(ph, power < 0);
    pwm_set_freq_duty(slice_num, channel, PWM_FREQ_HZ, fabs(power));
}

void hbridge_init(uint ph_pin, uint en_pin) {
    ph = ph_pin;

    gpio_out_init(NSLEEP_PIN);

    gpio_out_init(DRVOFF_PIN);
    gpio_put(DRVOFF_PIN, 1);

    gpio_set_function(en_pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(en_pin);
    channel = pwm_gpio_to_channel(en_pin);

    gpio_out_init(ph_pin);

    // Start wakeup procedure
    gpio_put(NSLEEP_PIN, 1);

    sleep_us(500);

    // Issue nSleep reset pulse
    gpio_put(NSLEEP_PIN, 0);
    sleep_us(50);
    gpio_put(NSLEEP_PIN, 1);

    gpio_put(DRVOFF_PIN, 0);

    // PWM enable
    pwm_set_enabled(slice_num, true);
    pwm_set_freq_duty(slice_num, channel, PWM_FREQ_HZ, 0.0f);
}
