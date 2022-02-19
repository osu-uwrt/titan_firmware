#include "pico/time.h"

#include "drivers/safety.h"
#include "hw/balancer_adc.h"
#include "hw/dshot.h"
#include "hw/esc_pwm.h"
#include "tasks/lowbatt.h"

bool lowbatt_initialized;

static bool port_battery_sensed = false;
static bool stbd_battery_sensed = false;
static bool prev_lowbatt_state = false;
static absolute_time_t earliest_reenable_time;

void lowbatt_tick(void) {
    hard_assert_if(LIFETIME_CHECK, !lowbatt_initialized);

    if (balancer_adc_initialized && !balancer_adc_readng_stale) {
        safety_lower_fault(FAULT_LOWBATT_STALE);

        // Check for existance of batteries
        // This is for if only one battery (or power supply) is going to power the bot
        // In this case it won't be powered from startup
        // But, if a battery drops off suddenly, it should still go into low voltage
        if (!port_battery_sensed && balancer_adc_get_port_voltage() >= LOWBATT_PRESENT_VOLTAGE){
            port_battery_sensed = true;
        }
        if (!stbd_battery_sensed && balancer_adc_get_stbd_voltage() >= LOWBATT_PRESENT_VOLTAGE) {
            stbd_battery_sensed = true;
        }

        bool in_low_battery = false;
        if (port_battery_sensed && balancer_adc_get_port_voltage() <= LOWBATT_CUTOFF_VOLTAGE) {
            in_low_battery = true;
        }
        if (stbd_battery_sensed && balancer_adc_get_stbd_voltage() <= LOWBATT_CUTOFF_VOLTAGE) {
            in_low_battery = true;
        }

        if (in_low_battery != prev_lowbatt_state) {
            if (in_low_battery) {
                prev_lowbatt_state = true;

                earliest_reenable_time = make_timeout_time_ms(LOWBATT_MIN_DISABLE_TIME);
                safety_raise_fault(FAULT_LOW_BATTERY);

#if HW_USE_DSHOT
                dshot_set_lowbatt(true);
#endif

#if HW_USE_PWM
                esc_pwm_set_lowbatt(true);
#endif
            }
            else if (absolute_time_diff_us(earliest_reenable_time, get_absolute_time()) >= 0) {
                prev_lowbatt_state = false;
                safety_lower_fault(FAULT_LOW_BATTERY);

#if HW_USE_DSHOT
                dshot_set_lowbatt(false);
#endif

#if HW_USE_PWM
                esc_pwm_set_lowbatt(true);
#endif
            }
        }
    } else {
        safety_raise_fault(FAULT_LOWBATT_STALE);
    }
}

void lowbatt_init(void) {
    hard_assert_if(LIFETIME_CHECK, lowbatt_initialized);
    lowbatt_initialized = true;
}
