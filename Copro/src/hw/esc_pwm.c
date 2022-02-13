#include <stdio.h>

#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/pwm_stamped.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "drivers/safety.h"
#include "hw/esc_pwm.h"

#define ESC_NEUTRAL_PWM_US 1500

bool esc_pwm_initialized = false;

bi_decl(bi_1pin_with_name(THRUSTER_1_PIN, "Thruster 1"));
bi_decl(bi_1pin_with_name(THRUSTER_2_PIN, "Thruster 2"));
bi_decl(bi_1pin_with_name(THRUSTER_3_PIN, "Thruster 3"));
bi_decl(bi_1pin_with_name(THRUSTER_4_PIN, "Thruster 4"));
bi_decl(bi_1pin_with_name(THRUSTER_5_PIN, "Thruster 5"));
bi_decl(bi_1pin_with_name(THRUSTER_6_PIN, "Thruster 6"));
bi_decl(bi_1pin_with_name(THRUSTER_7_PIN, "Thruster 7"));
bi_decl(bi_1pin_with_name(THRUSTER_8_PIN, "Thruster 8"));
static const uint thruster_pins[] = {
    THRUSTER_1_PIN, THRUSTER_2_PIN, THRUSTER_3_PIN, THRUSTER_4_PIN,
    THRUSTER_5_PIN, THRUSTER_6_PIN, THRUSTER_7_PIN, THRUSTER_8_PIN
};

#define set_thruster_pin(pin, val)  pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), val)
#define set_thruster_id(id, val)   do{ invalid_params_if(ESC_PWM, id > 8 || id < 1); set_thruster_pin(thruster_pins[id - 1], val); } while(0);

/**
 * @brief The alarm id of the active timeout alarm.
 * Should only be positive when an alarm is active
 */
static alarm_id_t esc_pwm_timeout_alarm_id = 0;

/**
 * @brief Timeout for when thrusters are allowed to run.
 * If this is in the future, thrusters should not be written to.
 * Gives time for thruster startup chime to play and power rails to settle
 */
static absolute_time_t esc_pwm_time_thrusters_allowed;

/**
 * @brief When set to true, should not let commands go to thrusters
 * Set to disable thrusters during low battery condition
 */
static bool esc_pwm_thruster_lowbatt_disable = false;


void esc_pwm_stop_thrusters(void) {
    // This command needs to be able to be called from kill switch callbacks
    // So this can be called at any point, so just ignore call if dshot is not initialized yet
    if (esc_pwm_initialized) {
        for (int i = 1; i <= 8; i++){
            set_thruster_id(i, ESC_NEUTRAL_PWM_US);
        }

        if (esc_pwm_timeout_alarm_id > 0) {
            cancel_alarm(esc_pwm_timeout_alarm_id);
            esc_pwm_timeout_alarm_id = 0;
        }
    }
}

/**
 * @brief Timeout function which fires if pwm commands have not been updated in enough time
 * 
 * @param id Active alarm ID
 * @param user_data not used
 * @return int64_t Reschedule time
 */
static int64_t esc_pwm_update_timeout(__unused alarm_id_t id, __unused void *user_data) {
    esc_pwm_stop_thrusters();
    esc_pwm_time_thrusters_allowed = make_timeout_time_ms(ESC_PWM_UPDATE_DISABLE_TIME_MS);
    safety_raise_fault(FAULT_THRUSTER_TIMEOUT);
    printf("Thrusters Timed Out\n");
    return 0;
}

void esc_pwm_update_thrusters(const riptide_msgs2__msg__PwmStamped *thruster_commands) {
    hard_assert_if(LIFETIME_CHECK, !esc_pwm_initialized);

    // Don't run thrusters if kill is being asserted
    if (safety_kill_get_asserting_kill()) {
        return;
    }

    // Make sure time is synchronized with network before attempting to compare timestamps
    if (!rmw_uros_epoch_synchronized()){
        printf("ESC PWM No Time Synchronization for Comand Verification\n");
        safety_raise_fault(FAULT_ROS_SOFT_FAIL);
        return;
    }

    // Check to make sure message isn't old
    int64_t command_time = (((int64_t)thruster_commands->header.stamp.sec) * 1000) + 
                            (thruster_commands->header.stamp.nanosec / 1000000);
    int64_t command_time_diff = rmw_uros_epoch_millis() - command_time;

    if (command_time_diff > ESC_PWM_COMMAND_MAX_TIME_DIFF_MS || command_time_diff < -ESC_PWM_COMMAND_MAX_TIME_DIFF_MS) {
        printf("Stale PWM command received: %lld ms old\n", command_time_diff);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // If a timeout alarm is scheduled, cancel it since there's a new update
    if (esc_pwm_timeout_alarm_id > 0) {
        cancel_alarm(esc_pwm_timeout_alarm_id);
        esc_pwm_timeout_alarm_id = 0;
    }

    // Thrusters shouldn't move if they have a timeout applied
    if (absolute_time_diff_us(esc_pwm_time_thrusters_allowed, get_absolute_time()) < 0) {
        return;
    }

    // Thrusters shouldn't move if safety kill is disabled
    if (absolute_time_diff_us(safety_kill_get_last_change(), get_absolute_time()) < ESC_PWM_WAKEUP_DELAY_MS*1000) {
        return;
    }

    // Thrusters shouldn't move if in low battery state
    if (esc_pwm_thruster_lowbatt_disable) {
        return;
    }

    // Only schedule timeout alarm if thrusters will actually be enabled
    bool needs_timeout_scheduled = false;

    #define set_thruster_with_check(name, val)  if(val != 1500){needs_timeout_scheduled=true;}\
                                                set_thruster_name(name, val);
        
    for (int i = 0; i < 8; i++){
        uint16_t val = thruster_commands->pwm[i];
        if (val < 1100 || val > 1900) {
            printf("Invalid Thruster Command Sent: %d on Thruster %d", thruster_commands[i], i+1);
            safety_raise_fault(FAULT_ROS_BAD_COMMAND);
            esc_pwm_stop_thrusters();
            return;
        }

        if (val != 1500) {
            needs_timeout_scheduled = true;
        }

        set_thruster_id(i + 1, val);
    }

    if (needs_timeout_scheduled) {
        esc_pwm_timeout_alarm_id = add_alarm_in_ms(ESC_PWM_MIN_UPDATE_RATE_MS, &esc_pwm_update_timeout, NULL, true);
        hard_assert(esc_pwm_timeout_alarm_id > 0);
    }
}

void esc_pwm_set_lowbatt(bool in_lowbatt_state) {
    // Note that the throttling to prevent constantly enabling/disabling is handled in lowbatt.c
    esc_pwm_thruster_lowbatt_disable = in_lowbatt_state;
    if (in_lowbatt_state) {
        esc_pwm_stop_thrusters();
    }
}

void esc_pwm_init(void) {
    hard_assert_if(LIFETIME_CHECK, esc_pwm_initialized);

    pwm_config config = pwm_get_default_config();

    // Set pwm to count every microsecond
    assert((clock_get_hz(clk_sys) % 1000000) == 0);
    pwm_config_set_clkdiv_int(&config, clock_get_hz(clk_sys) / 1000000);

    // Set pwm to cycle every 2500us (Update at 400 Hz)
    pwm_config_set_wrap(&config, 2500);

    // Initialized pins
    uint32_t initialized_slices = 0;
    static_assert(NUM_PWM_SLICES <= 32, "Too many slices to fit into counter");
    for (int i = 0; i < 8; i++){
        uint thruster_pin = thruster_pins[i];
        uint slice_num = pwm_gpio_to_slice_num(thruster_pin);
        uint channel = pwm_gpio_to_channel(thruster_pin);   

        // Initialize slice if needed
        if (!(initialized_slices & (1<<slice_num))) {
            initialized_slices |= (1<<slice_num);

            pwm_init(slice_num, &config, false);
        }
        
        // Initialize channel and pin
        pwm_set_chan_level(slice_num, channel, ESC_NEUTRAL_PWM_US);
        gpio_set_function(thruster_pin, GPIO_FUNC_PWM);
    }


    // Finally enable requested pwm slices
    int slice_num = 0;
    while (initialized_slices != 0){
        if (initialized_slices & 1){
            pwm_set_enabled(slice_num, true);
        }
        slice_num++;
        initialized_slices >>= 1;
    }

    esc_pwm_initialized = true;
}