#include <stdbool.h>
#include <rcl/rcl.h>
#include "can_mcp251Xfd/canbus.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define LED_BLINK_CYCLE_MS 2000
#define set_led_pin(pin, val)  pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), val)

struct led_status { 
    bool fault;
    bool can;
    bool enabled;
    bool killswitch;
};

static struct led_status status = {0};

void led_init() { 
    pwm_config config = pwm_get_default_config();

    assert((clock_get_hz(clk_sys) % 1000) == 0);
    pwm_config_set_clkdiv_int(&config, clock_get_hz(clk_sys) / 1000);
    pwm_config_set_wrap(&config, LED_BLINK_CYCLE_MS);

    uint32_t initialized_slices = 0;
    uint led_pins[] = {STATUS_LEDR_PIN, STATUS_LEDG_PIN, STATUS_LEDB_PIN};

    static_assert(NUM_PWM_SLICES <= 32, "Too many slices to fit into counter");
    for (unsigned int i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); i++){
        uint pin = led_pins[i];
        uint slice_num = pwm_gpio_to_slice_num(pin);
        uint channel =  pwm_gpio_to_channel(pin);

        // Initialize slice if needed
        if (!(initialized_slices & (1<<slice_num))) {
            initialized_slices |= (1<<slice_num);
            
            pwm_init(slice_num, &config, false);
        }

        // Initialize channel and pin
        set_led_pin(pin, 0);
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }

    int slice_num = 0;
    while (initialized_slices != 0){
        if (initialized_slices & 1){
            pwm_set_enabled(slice_num, true);
        }
        slice_num++;
        initialized_slices >>= 1;
    }

    bool ledr_channel_a = pwm_gpio_to_channel(STATUS_LEDR_PIN) == PWM_CHAN_A;
    pwm_set_output_polarity(pwm_gpio_to_slice_num(STATUS_LEDR_PIN), ledr_channel_a, !ledr_channel_a);
}

void led_update_pins() { 
    // Color Calculation: 
    // 1. Kill switch inserted: Blue
    // 2. Safety Initialized: Green
    // 3. CAN Online: Yellow
    // 4. CAN Offline: Red
    // If a fault occurs, it will alternate between the color above and red

    bool r = false;
    bool g = false;
    bool b = false;

    if(status.killswitch) {
        b = true; 
    } else if(status.enabled) {
        g = true;
    } else if(status.can) { 
        r = true;
        g = true;
    } else { 
        r = true;
    }

    if(status.fault) { 
        set_led_pin(STATUS_LEDR_PIN, status.fault ? 0 : LED_BLINK_CYCLE_MS / 2);
        set_led_pin(STATUS_LEDG_PIN, g ? LED_BLINK_CYCLE_MS / 2 : 0);
        set_led_pin(STATUS_LEDB_PIN, b ? LED_BLINK_CYCLE_MS / 2 : 0);
    } else { 
        set_led_pin(STATUS_LEDR_PIN, r ? 0 : LED_BLINK_CYCLE_MS);
        set_led_pin(STATUS_LEDG_PIN, g ? LED_BLINK_CYCLE_MS : 0);
        set_led_pin(STATUS_LEDB_PIN, b ? LED_BLINK_CYCLE_MS : 0);
    }
}

void led_fault_set(bool value) {
    status.fault = value;

    led_update_pins();
}  

void led_can_set(bool value) {
    status.can = value;

    led_update_pins();
}

void led_enabled_set(bool value) {
    status.enabled = value;

    led_update_pins();
}

void led_killswitch_set(bool value) {
    status.killswitch = value;

    led_update_pins();
}

rcl_ret_t led_update_can() { 
    led_can_set(canbus_check_online());

    return RCL_RET_OK;
}