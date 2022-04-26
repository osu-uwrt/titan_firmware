#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include "basic_logger/logging.h"

#include "actuators/claw.h"
#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "claw"

static const uint forward_direction_pin = CLAW_FORWARD_PIN;
static const uint reverse_direction_pin = CLAW_REVERSE_PIN;
static const uint enable_pin = CLAW_ENABLE_PIN;

#define OPEN_DIRECTION_IS_FORWARD true

bool claw_initialized = false;
static enum claw_state local_claw_state = CLAW_STATE_UNKNOWN_POSITION;

void claw_initialize(void) {
    hard_assert_if(LIFETIME_CHECK, claw_initialized);

    // Init GPIO
    gpio_init(enable_pin);
    gpio_put(enable_pin, false);
    gpio_set_dir(enable_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(enable_pin, "Claw Motor Enable"));

    gpio_init(forward_direction_pin);
    gpio_put(forward_direction_pin, false);
    gpio_set_dir(forward_direction_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(forward_direction_pin, "Claw Motor Forward"));

    gpio_init(reverse_direction_pin);
    gpio_put(reverse_direction_pin, false);
    gpio_set_dir(reverse_direction_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(reverse_direction_pin, "Claw Motor Reverse"));
    
    // Set state
    local_claw_state = CLAW_STATE_UNKNOWN_POSITION;

    claw_initialized = true;
}

// ========================================
// Timing Management
// ========================================

// A time value of 0 means uninitialized
static uint16_t claw_open_time_ms = 0;
static uint16_t claw_close_time_ms = 0;

enum actuator_command_result claw_set_timings(struct claw_timing_cmd *timings) {
    hard_assert_if(LIFETIME_CHECK, !claw_initialized);

    if (timings->open_time_ms == 0 || timings->close_time_ms == 0) {
        return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Setting claw timing (Open %d ms, Close %d ms)", timings->open_time_ms, timings->close_time_ms);

    claw_open_time_ms = timings->open_time_ms;
    claw_close_time_ms = timings->close_time_ms;

    return ACTUATOR_RESULT_SUCCESSFUL;
}

void claw_populate_missing_timings(struct missing_timings_status* missing_timings) {
    hard_assert_if(LIFETIME_CHECK, !claw_initialized);

    missing_timings->claw_open_timing = (claw_open_time_ms == 0);
    missing_timings->claw_close_timing = (claw_close_time_ms == 0);
}


// ========================================
// H-Bridge Control Functions
// ========================================

/**
 * @brief Sets the claw driver to move in the specified direction
 * 
 * @param forward Set to true to move forward, false to move in reverse
 */
static inline void claw_driver_move(bool forward) {
    gpio_put(forward_direction_pin, forward);
    gpio_put(reverse_direction_pin, !forward);
    gpio_put(enable_pin, true);
}

/**
 * @brief Stops the motor driver
 */
static inline void claw_driver_stop(void) {
    gpio_put(forward_direction_pin, false);
    gpio_put(reverse_direction_pin, false);
    gpio_put(enable_pin, false);
}

// ========================================
// Movement Management
// ========================================

static alarm_id_t scheduled_alarm_id = 0;

/**
 * @brief Internal shared function to stop the claw and clean up the state
 * 
 * @param target_state The state of the claw after stopping
 */
void claw_stop_internal(enum claw_state target_state) {
    claw_driver_stop();
    scheduled_alarm_id = 0;
    local_claw_state = target_state;
}

/**
 * @brief Alarm for when to finish moving the claw
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is the target claw state
 * @return int64_t If/How to restart the timer
 */
static int64_t claw_finish_callback(__unused alarm_id_t id, void *user_data) {
    enum claw_state target_state = ((enum claw_state) user_data);
    valid_params_if(CLAW, (target_state == CLAW_STATE_OPENED || target_state == CLAW_STATE_CLOSED));
    
    claw_stop_internal(target_state);

    return 0;
}

enum claw_state claw_get_state(void) {
    hard_assert_if(LIFETIME_CHECK, !claw_initialized);

    if (claw_open_time_ms == 0 || claw_close_time_ms == 0) {
        return CLAW_STATE_UNINITIALIZED;
    }

    return local_claw_state;
}

enum actuator_command_result claw_open(void) {
    hard_assert_if(LIFETIME_CHECK, !claw_initialized);

    if (safety_kill_get_asserting_kill()) {
        return ACTUATOR_RESULT_FAILED;
    }

    switch (claw_get_state()) {
        case CLAW_STATE_OPENED:
            return ACTUATOR_RESULT_SUCCESSFUL;
        case CLAW_STATE_OPENING:
            return ACTUATOR_RESULT_RUNNING;
        case CLAW_STATE_UNKNOWN_POSITION:
        case CLAW_STATE_CLOSED:
            break;
        default:
            return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Opening Claw");
    
    local_claw_state = CLAW_STATE_OPENING;
    scheduled_alarm_id = add_alarm_in_ms(claw_open_time_ms, &claw_finish_callback, ((void*)CLAW_STATE_OPENED), true);
    claw_driver_move(OPEN_DIRECTION_IS_FORWARD);

    return ACTUATOR_RESULT_RUNNING;
}

enum actuator_command_result claw_close(void) {
    hard_assert_if(LIFETIME_CHECK, !claw_initialized);

    if (safety_kill_get_asserting_kill()) {
        return ACTUATOR_RESULT_FAILED;
    }

    switch (claw_get_state()) {
        case CLAW_STATE_CLOSED:
            return ACTUATOR_RESULT_SUCCESSFUL;
        case CLAW_STATE_CLOSING:
            return ACTUATOR_RESULT_RUNNING;
        case CLAW_STATE_UNKNOWN_POSITION:
        case CLAW_STATE_OPENED:
            break;
        default:
            return ACTUATOR_RESULT_FAILED;
    }

    LOG_INFO("Closing Claw");

    local_claw_state = CLAW_STATE_CLOSING;
    scheduled_alarm_id = add_alarm_in_ms(claw_close_time_ms, &claw_finish_callback, ((void*)CLAW_STATE_CLOSED), true);
    claw_driver_move(!OPEN_DIRECTION_IS_FORWARD);

    return ACTUATOR_RESULT_RUNNING;
}

void claw_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since local_claw_state should not be changed until initialized

    if (local_claw_state == CLAW_STATE_OPENING || local_claw_state == CLAW_STATE_CLOSING) {
        cancel_alarm(scheduled_alarm_id);
        claw_stop_internal(CLAW_STATE_UNKNOWN_POSITION);
    }
}