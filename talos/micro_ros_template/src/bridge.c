#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define N_SLEEP_PIN 25
#define N_FAULT_PIN 11
#define PH_PIN 0
#define EN_PIN 1

#define T_READY_US 600
#define T_WAKEUP_US 10
#define T_COM_US 400
#define T_RESET_US 15

extern volatile int32_t latest_channel_value;
bool direction = true;
uint slice_num1;

void hbridge_init() {
    // PH pin
    gpio_init(PH_PIN);
    gpio_set_dir(PH_PIN, GPIO_OUT);

    // EN pin
    gpio_set_function(EN_PIN, GPIO_FUNC_PWM);
    slice_num1 = pwm_gpio_to_slice_num(EN_PIN);

    pwm_set_wrap(slice_num1, 6249);
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, latest_channel_value);  // PWM input
    pwm_set_enabled(slice_num1, true);

    latest_channel_value < 0 ? gpio_put(0, false) : gpio_put(0, true);

    gpio_init(N_SLEEP_PIN);
    gpio_init(N_FAULT_PIN);

    gpio_set_dir(N_SLEEP_PIN, GPIO_OUT);  // nSLEEP pin
    gpio_set_dir(N_FAULT_PIN, GPIO_IN);   // nFAULT pin

    gpio_put(N_SLEEP_PIN, true);

    sleep_us(T_WAKEUP_US);  // t_wakeup
    sleep_us(T_COM_US);     // t_com

    bool nfault_value = gpio_get(N_FAULT_PIN);

    if (nfault_value) {
        printf("Error occured during board bring up.");
    }

    sleep_us(T_READY_US);  // t_ready

    gpio_put(N_SLEEP_PIN, false);
    sleep_us(T_RESET_US);  // t_reset

    gpio_put(N_SLEEP_PIN, true);
}

void update_hbridge(int32_t latest_channel_value) {
    latest_channel_value < 0 ? gpio_put(PH_PIN, false) : gpio_put(PH_PIN, true);
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, abs(latest_channel_value));
    printf("Updated channel to %ld\n", abs(latest_channel_value));
}
