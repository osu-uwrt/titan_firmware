#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include "basic_logger/logging.h"

#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "claw"

#warning This file must be fixed before production use
// TODO: Fix this up to support new design of being interrupt safe and supporting checking actuator arm state

enum claw_state {
    CLAW_STATE_ERROR = riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR,
    CLAW_STATE_DISARMED = riptide_msgs2__msg__ActuatorStatus__CLAW_DISARMED,
    CLAW_STATE_UNKNOWN_POSITION = riptide_msgs2__msg__ActuatorStatus__CLAW_UNKNOWN,
    CLAW_STATE_OPENED = riptide_msgs2__msg__ActuatorStatus__CLAW_OPENED,
    CLAW_STATE_CLOSED = riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSED,
    CLAW_STATE_OPENING = riptide_msgs2__msg__ActuatorStatus__CLAW_OPENING,
    CLAW_STATE_CLOSING = riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSING
};

static const uint direction_pin = CLAW_PHASE_PIN;
static const uint mode2_pin = CLAW_MODE2_PIN;
static const uint enable_pin = CLAW_ENABLE_PIN;

#define OPEN_DIRECTION_IS_FORWARD true

static enum claw_state local_claw_state = CLAW_STATE_UNKNOWN_POSITION;

void claw_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);

    // Init GPIO
    gpio_init(enable_pin);
    gpio_put(enable_pin, false);
    gpio_set_dir(enable_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(enable_pin, "Claw Motor Enable"));

    gpio_init(direction_pin);
    gpio_put(direction_pin, false);
    gpio_set_dir(direction_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(direction_pin, "Claw Direction Pin"));

    gpio_init(mode2_pin);
    gpio_put(mode2_pin, false);
    gpio_set_dir(mode2_pin, true);
    bi_decl_if_func_used(bi_1pin_with_name(mode2_pin, "Claw Mode 2 Pin"));

    // Set state
    local_claw_state = CLAW_STATE_UNKNOWN_POSITION;
}

// ========================================
// Timing Management
// ========================================

// A time value of 0 means uninitialized
static uint16_t claw_open_time_ms = 0;
static uint16_t claw_close_time_ms = 0;

bool claw_set_timings(uint16_t open_time_ms, uint16_t close_time_ms) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (open_time_ms == 0 || close_time_ms == 0) {
        return false;
    }

    LOG_INFO("Setting claw timing (Open %d ms, Close %d ms)", open_time_ms, close_time_ms);

    claw_open_time_ms = open_time_ms;
    claw_close_time_ms = close_time_ms;

    return true;
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
    gpio_put(direction_pin, forward);
    gpio_put(enable_pin, true);
}

/**
 * @brief Stops the motor driver
 */
static inline void claw_driver_stop(void) {
    gpio_put(direction_pin, false);
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
static void claw_stop_internal(enum claw_state target_state) {
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
    valid_params_if(ACTUATORS, (target_state == CLAW_STATE_OPENED || target_state == CLAW_STATE_CLOSED));

    claw_stop_internal(target_state);

    return 0;
}

uint8_t claw_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (claw_open_time_ms == 0 || claw_close_time_ms == 0) {
        return CLAW_STATE_ERROR;
    }

    return local_claw_state;
}

bool claw_get_busy(void) {
    return scheduled_alarm_id != 0;
}

bool claw_open(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (safety_kill_get_asserting_kill()) {
        *errMsgOut = "Kill Pulled";
        return false;
    }

    switch (claw_get_state()) {
        case CLAW_STATE_OPENED:
        case CLAW_STATE_OPENING:
            return true;
        case CLAW_STATE_UNKNOWN_POSITION:
        case CLAW_STATE_CLOSED:
            break;
        default:
            *errMsgOut = "Invalid State";
            return false;
    }

    LOG_INFO("Opening Claw");

    local_claw_state = CLAW_STATE_OPENING;
    scheduled_alarm_id = add_alarm_in_ms(claw_open_time_ms, &claw_finish_callback, ((void*)CLAW_STATE_OPENED), true);
    claw_driver_move(OPEN_DIRECTION_IS_FORWARD);

    return true;
}

bool claw_close(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (safety_kill_get_asserting_kill()) {
        LOG_DEBUG("Not closing claw cuz safety");
        *errMsgOut = "Kill Pulled";
        return false;
    }

    switch (claw_get_state()) {
        case CLAW_STATE_CLOSED:
        case CLAW_STATE_CLOSING:
            return true;
        case CLAW_STATE_UNKNOWN_POSITION:
        case CLAW_STATE_OPENED:
            break;
        default:
            LOG_DEBUG("defualt case");
            return false;
    }

    LOG_INFO("Closing Claw");

    local_claw_state = CLAW_STATE_CLOSING;
    scheduled_alarm_id = add_alarm_in_ms(claw_close_time_ms, &claw_finish_callback, ((void*)CLAW_STATE_CLOSED), true);
    claw_driver_move(!OPEN_DIRECTION_IS_FORWARD);

    return true;
}

void claw_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since local_claw_state should not be changed until initialized

    if (local_claw_state == CLAW_STATE_OPENING || local_claw_state == CLAW_STATE_CLOSING) {
        cancel_alarm(scheduled_alarm_id);
        claw_stop_internal(CLAW_STATE_UNKNOWN_POSITION);
    }
    else {
        // We might not be moving, but be *very* sure that we've stopped the claw
        claw_driver_stop();
    }
}