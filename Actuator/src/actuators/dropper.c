#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include "basic_logger/logging.h"

#include "actuators/dropper.h"
#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dropper"


#define DROPPER_LEVEL_ON  1
#define DROPPER_LEVEL_OFF 0
struct dropper_data {
    bool dropping;
    bool dropped;
    uint pin_id;
    alarm_id_t stop_timer;
};

struct dropper_data dropper_data[] = {
    {   // Dropper 1
        .pin_id = DROPPER_1_PIN,
        .dropping = false, .dropped = false,
    },
    {   // Dropper 2
        .pin_id = DROPPER_2_PIN,
        .dropping = false, .dropped = false,
    }
};
#define NUM_DROPPERS (sizeof(dropper_data)/sizeof(*dropper_data))


bool dropper_initialized = false;

void dropper_initialize(void) {
    hard_assert_if(LIFETIME_CHECK, dropper_initialized);

    bi_decl_if_func_used(bi_1pin_with_name(DROPPER_1_PIN, "Dropper 1 Drop"));
    bi_decl_if_func_used(bi_1pin_with_name(DROPPER_2_PIN, "Dropper 2 Drop"));

    for (uint i = 0; i < NUM_DROPPERS; i++) {
        uint pin = dropper_data[i].pin_id;
        gpio_init(pin);
        gpio_put(pin, DROPPER_LEVEL_OFF);
        gpio_set_dir(pin, true);
    }

    dropper_initialized = true;
}


// ========================================
// Timing Management
// ========================================

// A time value of 0 means uninitialized
static uint16_t dropper_active_time_ms = 0;

enum actuator_command_result dropper_set_timings(struct dropper_timing_cmd *timings) {
    hard_assert_if(LIFETIME_CHECK, !dropper_initialized);

    if (timings->active_time_ms == 0){
        return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Setting dropper timings to %d ms", timings->active_time_ms);
    dropper_active_time_ms = timings->active_time_ms;
    return ACTUATOR_RESULT_SUCCESSFUL;
}

void dropper_populate_missing_timings(struct missing_timings_status* missing_timings) {
    hard_assert_if(LIFETIME_CHECK, !dropper_initialized);

    missing_timings->dropper_active_timing = (dropper_active_time_ms == 0);
}


// ========================================
// Movement Management
// ========================================

/**
 * @brief Internal shared function to stop the marker dropper and clean up the state
 * 
 * @param this_dropper The dropper to stop
 */
void dropper_stop_internal(struct dropper_data *this_dropper) {
    this_dropper->dropped = true;
    this_dropper->dropping = false;
    this_dropper->stop_timer = 0;

    gpio_put(this_dropper->pin_id, DROPPER_LEVEL_OFF);
}

/**
 * @brief Alarm for when to finish dropping the specific dropper
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is a pointer to the dropper_data struct
 * @return int64_t If/How to restart the timer
 */
static int64_t dropper_finish_callback(__unused alarm_id_t id, void *user_data) {
    dropper_stop_internal((struct dropper_data *)(user_data));

    return 0;
}


// ===== Public Functions =====

enum dropper_state dropper_get_state(uint8_t dropper_id) {
    hard_assert_if(LIFETIME_CHECK, !dropper_initialized);

    valid_params_if(DROPPER, dropper_id >= 1 && dropper_id <= NUM_DROPPERS);
    struct dropper_data *this_dropper = &dropper_data[dropper_id-1];

    if (dropper_active_time_ms == 0) {
        return DROPPER_STATE_UNINITIALIZED;
    }

    if (this_dropper->dropping) {
        return DROPPER_STATE_DROPPING;
    } else if (this_dropper->dropped) {
        return DROPPER_STATE_DROPPED;
    } else {
        return DROPPER_STATE_READY;
    }
}

enum actuator_command_result dropper_drop_marker(struct drop_marker_cmd *cmd) {
    hard_assert_if(LIFETIME_CHECK, !dropper_initialized);

    uint8_t dropper_num = cmd->dropper_num;
    if (dropper_num <= 0 || dropper_num > NUM_DROPPERS) {
        return ACTUATOR_RESULT_FAILED;
    }

    if (safety_kill_get_asserting_kill()) {
        return ACTUATOR_RESULT_FAILED;
    }

    switch (dropper_get_state(dropper_num)) {
        case DROPPER_STATE_DROPPED:
            return ACTUATOR_RESULT_SUCCESSFUL;
        case DROPPER_STATE_DROPPING:
            return ACTUATOR_RESULT_RUNNING;
        case DROPPER_STATE_READY:
            break;
        default:
            return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Dropping Marker %d", dropper_num);

    struct dropper_data *this_dropper = &dropper_data[dropper_num-1];
    this_dropper->dropping = true;
    this_dropper->stop_timer = add_alarm_in_ms(dropper_active_time_ms, dropper_finish_callback, this_dropper, true);
    hard_assert(this_dropper->stop_timer > 0);
    
    gpio_put(this_dropper->pin_id, DROPPER_LEVEL_ON);

    return ACTUATOR_RESULT_RUNNING;
}

enum actuator_command_result dropper_clear_status(void) {
    hard_assert_if(LIFETIME_CHECK, !dropper_initialized);

    LOG_INFO("Clearing Dropper Status");

    enum actuator_command_result result = ACTUATOR_RESULT_SUCCESSFUL;
    for (uint i = 0; i < NUM_DROPPERS; i++) {
        struct dropper_data *this_dropper = &dropper_data[i];

        if (this_dropper->dropping) {
            result = ACTUATOR_RESULT_FAILED;
        } else if (this_dropper->dropped){
            this_dropper->dropped = false;
        }
    }
    return result;
}

void dropper_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since this_dropper->dropping should never be true before init

    for (uint i = 0; i < NUM_DROPPERS; i++) {
        struct dropper_data *this_dropper = &dropper_data[i];

        if (this_dropper->dropping) {
            cancel_alarm(this_dropper->stop_timer);
            dropper_stop_internal(this_dropper);
        }
    }
}