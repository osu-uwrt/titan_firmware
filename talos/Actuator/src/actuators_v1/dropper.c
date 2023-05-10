#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/sync.h"
#include "pico/time.h"

#include "basic_logger/logging.h"

#include "actuators.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dropper"

// Pin Definitions
#define DROPPER_LEVEL_ON  1
#define DROPPER_LEVEL_OFF 0

bi_decl(bi_1pin_with_name(DROPPER_1_PIN, "Dropper 1 Drop"));
bi_decl(bi_1pin_with_name(DROPPER_2_PIN, "Dropper 2 Drop"));
static const uint dropper_pins[] = {DROPPER_1_PIN, DROPPER_2_PIN};
#define NUM_DROPPERS (sizeof(dropper_pins)/sizeof(*dropper_pins))

// A time value of 0 means uninitialized
static uint16_t dropper_active_time_ms = 0;

/**
 * @brief Pin mask of all dropper pins, so all droppers can be cleared quickly
 */
static uint32_t dropper_pin_mask = 0;
static uint32_t dropper_off_mask = 0;

/**
 * @brief The index of the next dropper to fire
 * If == NUM_DROPPERS, all the droppers have fired
 */
static volatile uint next_dropper_index = 0;

/**
 * @brief Boolean value if a dropper alarm is active.
 * Needed since dropper_safety_disable can be called at *any* time, since it's part of the killswitch system.
 * To ensure the droppers promptly stop, they will abort the active dropping sequence.
 *
 * But since cancelling alarms gets very hairy depending on what you're doing (if kill switch pulled while the alarm
 * is currently being dispatched, and cancel is called on a currently firing alarm, the SDK gets a little squirly)
 * So instead we're just going to report we're dropping until the alarm fires *but* it will have technically stopped
 * by the kill switch event.
 *
 * The torpedo was done differently though since it's a lot easier to clean up a PIO machine (just write a few registers)
 * rather than cancel an alarm (they've got heaps and a bunch of fun stuff underneath the alarms which I don't want to
 * play games with)
 *
 * So this *technically* isn't the boolean if the dropper is dropping, but rather a boolean if the alarm is firing.
 * Now we'll report this as a status of dropping, but there is technically a difference due to the kill switch.
 */
static volatile bool dropper_alarm_active = false;

// ========================================
// Internal Functions
// ========================================

void dropper_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);

    // Initialize mask variable
    for (uint i = 0; i < NUM_DROPPERS; i++) {
        dropper_pin_mask |= 1 << dropper_pins[i];
        dropper_off_mask |= DROPPER_LEVEL_OFF << dropper_pins[i];
    }

    // Initialize all the dropper pins
    gpio_init_mask(dropper_pin_mask);
    gpio_put_masked(dropper_pin_mask, dropper_off_mask);
    gpio_set_dir_out_masked(dropper_pin_mask);
}

void dropper_safety_disable(void) {
    // It should be fine that this be called even before initialization
    // since dropper_pin_mask will be 0 before init

    gpio_put_masked(dropper_pin_mask, dropper_off_mask);

    if (dropper_alarm_active) {
        // Report that the actuation was aborted
        // This is important since we responed over ROS that the actuator fired, but now we're cancelling it
        // Due to how brief these pulses are, it is worth the occasional fault when we pull the kill switch over thinking
        // the actuator fired and some random kill event stopped it silently (like a timeout)
        safety_raise_fault(FAULT_ACTUATOR_FAILURE);
    }
}

// ========================================
// Timing Management
// =======================================

bool dropper_set_timings(uint16_t active_time_ms) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (active_time_ms == 0){
        return false;
    }

    if(actuators_armed) {
        return false;
    }

    LOG_INFO("Setting dropper timings to %d ms", active_time_ms);
    dropper_active_time_ms = active_time_ms;
    return true;
}

// ========================================
// Movement Management
// ========================================

/**
 * @brief Alarm for when to finish dropping the specific dropper
 *
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. Unused
 * @return int64_t If/How to restart the timer
 */
static int64_t dropper_finish_callback(__unused alarm_id_t id, __unused void *user_data) {

    // Clear the dropper pins
    gpio_put_masked(dropper_pin_mask, dropper_off_mask);

    // Mark that the dropper has dropped
    next_dropper_index++;
    dropper_alarm_active = false;

    return 0;
}


// ===== Public Functions =====

uint8_t dropper_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (dropper_active_time_ms == 0) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
    } else if(!actuators_armed) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DISARMED;
    } if (dropper_alarm_active) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPING;
    } else {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_READY;
    }
}

bool dropper_get_busy(void) {
    return dropper_alarm_active;
}

uint8_t dropper_get_available(void) {
    int avail = NUM_DROPPERS - next_dropper_index;
    return (avail > 0 ? avail : 0);
}


bool dropper_drop_marker(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (dropper_active_time_ms == 0) {
        *errMsgOut = "Missing Timings";
        return false;
    }

    if (next_dropper_index >= NUM_DROPPERS) {
        *errMsgOut = "All Droppers Fired";
    }

    // Critical section so that a kill switch event won't fire between checking armed & setting the GPIOs
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Check volatile fields in critical section
    if (safety_kill_get_asserting_kill()) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Kill Switch Pulled";
        return false;
    }

    if(!actuators_armed)  {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Not Armed";
        return false;
    }

    // Drop the next dropper in the line
    uint fired_dropper = next_dropper_index;
    dropper_alarm_active = true;
    gpio_put(dropper_pins[fired_dropper], DROPPER_LEVEL_ON);

    restore_interrupts(prev_interrupts);

    // We can schedule the alarm outside of the critical section since the kill switch won't cancel the alarm
    // and adding alarms takes a non-insignificant amount of time (especially taking into account cache misses)

    // And hard assert if we couldn't schedule the alarm due to running out of timers (-1 is an out of timer error)
    hard_assert(add_alarm_in_ms(dropper_active_time_ms, dropper_finish_callback, NULL, true) >= 0);

    LOG_INFO("Dropping Marker %d", fired_dropper+1);

    return true;
}

bool dropper_notify_reload(const char **errMsgOut) {
    if (dropper_alarm_active) {
        *errMsgOut = "Dropper Active";
        return false;
    }

    LOG_INFO("Resetting Dropper Fire Count");

    next_dropper_index = 0;
    return true;
}
