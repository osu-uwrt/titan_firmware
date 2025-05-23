#include "hbridge.h"

#include "safety_interface.h"

#include "driver/cd74hc4051.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "titan/logger.h"

#include <math.h>
#include <stdlib.h>

#define PWM_FREQ_HZ 25000

#define WAKEUP_TIME_FOR_COM_US 500
#define WAKEUP_TIME_RESET_US 50
#define TIME_SLEEP_US 120

#define MAX_WAKE_ATTEMPTS 3

#define SLEW_PERIOD_MS 10
#define SLEW_MAX_DIFF 0.05f

#define PCT_MAX 1.0f
#define PCT_MIN -1.0f

typedef struct hbridge_t {
    uint ph_pin;

    uint en_chan;
    uint en_slice;
    uint16_t en_wrap_value;

    uint nfault_access;
    bool multiplex_nfault;

    volatile float target_pct;
    float curr_pct;

    float invert;
} hbridge;

uint all_nsleep_pin;
uint all_drvoff_pin;

hbridge *bridges;
uint max_num_bridges = 0;
uint bridge_cnt = 0;
bool bridges_enabled = false;

static repeating_timer_t slew_timer;

static void gpio_out_init(uint pin, bool start_value) {
    gpio_init(pin);
    gpio_put(pin, start_value);
    gpio_set_dir(pin, GPIO_OUT);
}

static void hbridge_pwm_init(uint pin, hbridge *bridge) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    bridge->en_slice = pwm_gpio_to_slice_num(pin);
    bridge->en_chan = pwm_gpio_to_channel(pin);

    // Compute PWM parameters based on frequency
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = clock / PWM_FREQ_HZ / 4096 + (clock % (PWM_FREQ_HZ * 4096) != 0);

    if (divider16 / 16 == 0)
        divider16 = 16;

    uint32_t wrap = clock * 16 / divider16 / PWM_FREQ_HZ - 1;

    pwm_set_clkdiv_int_frac(bridge->en_slice, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(bridge->en_slice, wrap);
    bridge->en_wrap_value = wrap;

    // Enable PWM, but set chan level to 0 to be off initially
    pwm_set_chan_level(bridge->en_slice, bridge->en_chan, 0);
    pwm_set_enabled(bridge->en_slice, true);
}

static void hbridge_set_curr_duty(hbridge *bridge) {
    gpio_put(bridge->ph_pin, bridge->curr_pct < 0.0f);
    pwm_set_chan_level(bridge->en_slice, bridge->en_chan, bridge->en_wrap_value * fabs(bridge->curr_pct));
}

static bool hbridge_slew(__unused repeating_timer_t *rt) {
    for (uint i = 0; i < bridge_cnt; i++) {
        float error = bridges[i].target_pct - bridges[i].curr_pct;

        if (fabs(error) < SLEW_MAX_DIFF)
            bridges[i].curr_pct = bridges[i].target_pct;
        else
            // Error can't be 0 here
            bridges[i].curr_pct += SLEW_MAX_DIFF * (error / fabs(error));

        hbridge_set_curr_duty(&bridges[i]);
    }

    return true;
}

void hbridge_set_target(uint idx, float target_pct) {
    if (!bridges_enabled)
        return;

    LOG_INFO("Set a new target of %f", target_pct);

    // Scale to [-1, 1]
    bridges[idx].target_pct = MIN(PCT_MAX, MAX(PCT_MIN, target_pct));
}

static bool hbridge_get_nfault(hbridge *bridge) {
    if (bridge->multiplex_nfault)
        return multiplexer_decode_digital(bridge->nfault_access);
    else
        return gpio_get(bridge->nfault_access);
}

void hbridge_set_enabled(bool enabled) {
    for (uint i = 0; i < bridge_cnt; i++) {
        bridges[i].target_pct = 0.0f;
    }

    gpio_put(all_drvoff_pin, !enabled);
    bridges_enabled = enabled;
}

// Returns the number of bridges woken
uint hbridge_wake() {
    uint num_awake = 0;
    uint num_attempts = 0;

    bool wake_tracking_arr[bridge_cnt];

    while (num_awake < bridge_cnt && num_attempts < MAX_WAKE_ATTEMPTS) {
        num_awake = 0;

        // Put bridges into sleep
        gpio_put(all_nsleep_pin, 0);
        sleep_us(TIME_SLEEP_US);

        // Start wakeup procedure
        gpio_put(all_nsleep_pin, 1);
        sleep_us(WAKEUP_TIME_FOR_COM_US);

        // Verify nFault is low
        for (uint i = 0; i < bridge_cnt; i++) {
            wake_tracking_arr[i] = !hbridge_get_nfault(&bridges[i]);
        }

        // Wait for bridges to be ready. This sleep is VERY IMPORTANT
        sleep_us(WAKEUP_TIME_FOR_COM_US);

        // Issue nSleep reset pulse
        gpio_put(all_nsleep_pin, 0);
        sleep_us(WAKEUP_TIME_RESET_US);
        gpio_put(all_nsleep_pin, 1);

        // Verify nFault is high
        for (uint i = 0; i < bridge_cnt; i++) {
            if (hbridge_get_nfault(&bridges[i]) && wake_tracking_arr[i])
                num_awake++;
        }

        num_attempts++;
    }

    if (num_awake < bridge_cnt) {
        // Issue a fault with the number of bridges missing as data
        safety_raise_fault_with_arg(FAULT_HBRIDGE_DISCONNECTED, bridge_cnt - num_awake);
        LOG_WARN("Only %d hbridges found. Expected %d", num_awake, bridge_cnt);
    }
    else {
        safety_lower_fault(FAULT_HBRIDGE_DISCONNECTED);
    }

    return num_awake;
}

void hbridge_sleep() {
    gpio_put(all_nsleep_pin, 0);
}

// Returns the bridge index
uint hbridge_create(uint ph_pin, uint en_pin, bool invert, uint nfault_access, bool multiplex_nfault) {
    if (bridge_cnt >= max_num_bridges) {
        LOG_ERROR("Too many bridges created. Only %u specified by hbridge_init.", max_num_bridges);
        return -1;
    }

    hbridge new_bridge = { .ph_pin = ph_pin,
                           .nfault_access = nfault_access,
                           .multiplex_nfault = multiplex_nfault,
                           .target_pct = 0.0f,
                           .curr_pct = 0.0f,
                           .invert = invert ? -1.0f : 1.0f };

    gpio_out_init(ph_pin, 0);
    hbridge_pwm_init(en_pin, &new_bridge);

    if (!multiplex_nfault) {
        gpio_init(nfault_access);
    }

    bridges[bridge_cnt] = new_bridge;
    return bridge_cnt++;
}

void hbridge_init(uint num_bridges, uint nsleep_pin, uint drvoff_pin) {
    bridges = malloc(num_bridges * sizeof(hbridge));
    max_num_bridges = num_bridges;

    all_nsleep_pin = nsleep_pin;
    all_drvoff_pin = drvoff_pin;

    gpio_out_init(nsleep_pin, 0);
    gpio_out_init(drvoff_pin, 1);

    add_repeating_timer_ms(SLEW_PERIOD_MS, hbridge_slew, NULL, &slew_timer);
}
