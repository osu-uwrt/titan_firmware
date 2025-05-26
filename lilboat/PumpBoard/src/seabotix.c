#include "seabotix.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "titan/logger.h"

#include <stdlib.h>

#define NUM_MOTORS 2

uint pos_slice_num[NUM_MOTORS];
uint pos_chan[NUM_MOTORS];

uint neg_slice_num[NUM_MOTORS];
uint neg_chan[NUM_MOTORS];

static uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d) {
    uint32_t clock = clock_get_hz(clk_sys);
    // uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);

    if (divider16 / 16 == 0)
        divider16 = 16;

    uint32_t wrap = clock * 16 / divider16 / f - 1;

    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);

    return wrap;
}

void seabotix_set_pct(uint8_t target, int8_t pct) {
    // Clamp pct on [-100, 100]
    pct = MIN(MAX(pct, -100), 100);

    float pos_duty = 100.0f;
    float neg_duty = 100.0f;

    if (abs(pct) < 1) {
        pos_duty = 0.0f;
        neg_duty = 0.0f;
    }
    else if (pct > 0)
        neg_duty = 100.0f - pct;
    else
        pos_duty = 100.0f - abs(pct);

    pwm_set_freq_duty(pos_slice_num[target], pos_chan[target], SEABOTIX_PWM_HZ, pos_duty);
    pwm_set_freq_duty(neg_slice_num[target], neg_chan[target], SEABOTIX_PWM_HZ, neg_duty);

    // LOG_INFO("Setting target %d to %f positive and %f negative", target, pos_duty, neg_duty);
}

static int64_t seabotix_stop_callback(__unused alarm_id_t id, void *user_data) {
    int8_t *target_ptr = ((int8_t *) user_data);
    seabotix_set_pct(*target_ptr, 0);

    free(target_ptr);
    return 0;
}

void seabotix_set_pct_for(uint8_t target, int8_t pct, uint time_ms) {
    seabotix_set_pct(target, pct);

    int8_t *target_persistent = malloc(sizeof(int8_t));
    *target_persistent = target;
    add_alarm_in_ms(time_ms, seabotix_stop_callback, target_persistent, false);
}

static void init_pin(uint pin) {
    // gpio_init(pin);
    // gpio_put(pin, 0);
    // gpio_set_dir(pin, GPIO_OUT);
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

void seabotix_init() {
    init_pin(MOT0_2_PIN);
    init_pin(MOT0_1_PIN);

    init_pin(MOT1_2_PIN);
    init_pin(MOT1_1_PIN);

    neg_slice_num[0] = pwm_gpio_to_slice_num(MOT0_2_PIN);
    neg_chan[0] = pwm_gpio_to_channel(MOT0_2_PIN);

    pos_slice_num[0] = pwm_gpio_to_slice_num(MOT0_1_PIN);
    pos_chan[0] = pwm_gpio_to_channel(MOT0_1_PIN);

    neg_slice_num[1] = pwm_gpio_to_slice_num(MOT1_2_PIN);
    neg_chan[1] = pwm_gpio_to_channel(MOT1_2_PIN);
    pos_slice_num[1] = pwm_gpio_to_slice_num(MOT1_1_PIN);
    pos_chan[1] = pwm_gpio_to_channel(MOT1_1_PIN);

    seabotix_set_pct(0, 0);
    seabotix_set_pct(1, 0);

    seabotix_set_enable_state(true);
}

void seabotix_disable() {
    seabotix_set_enable_state(false);
}

void seabotix_set_enable_state(bool state) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        pwm_set_enabled(pos_slice_num[i], state);
        pwm_set_enabled(neg_slice_num[i], state);
    }
}
