#include "titan/logger.h"

#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo_marker"

// Location Definitions
/**
 * @brief The servo position corresponding to the rest position of the servo
 */
#define POSITION_HOME 2472

/**
 * @brief Margin (in each direction) from HOME_POSITION where the torpedo is considered at home
 * Used to determine if the torpedo is home where it is supposed to be, before attempting another command
 *
 * 64: ~5.625 deg in each direction / 11.25 deg total
 */
#define HOME_MARGIN 64

static_assert(POSITION_HOME + HOME_MARGIN < 4096, "Margin overflows");
static_assert(POSITION_HOME - HOME_MARGIN >= 0, "Margin overflows");

/**
 * @brief Positions for various fire targets
 */
#define POSITION_TORPEDO1_FIRE 1448
#define POSITION_TORPEDO2_FIRE 444
#define POSITION_DROPPER1_FIRE 3496
#define POSITION_DROPPER2_FIRE 4095

#define MAX_MOVEMENT_TIME_MS 5000

// Index tracking
// These variables are not safe to modify in interrupts
static uint torpedo_next_index = 0;
static uint dropper_next_index = 0;

// Internal state tracking
// These variables are safe to modify in interrupts
static dynamixel_id torpedo_marker_id;
static volatile bool torpedo_marker_connected = false;
static volatile bool torpedo_marker_moving = false;
static volatile bool torpedo_marker_enabled = false;
static volatile bool torpedo_marker_homed = false;
static volatile bool torpedo_marker_hardware_err = false;

// Only valid when torpedo_marker_moving = true
static int32_t torpedo_marker_target_position = 0;
static absolute_time_t torpedo_marker_move_timeout;

// ========================================
// Movement Management
// ========================================

static bool torpedo_marker_set_target(const char **errMsgOut, int32_t target_position) {
    if (!torpedo_marker_connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    // Check both, as enabled reflects the actual dynamixel state, which might be cleared if an error occurs on the
    // dynamixel, while actuators_armed reflects if safety is permitting actuator movement, which might not be reflected
    // on the dynamixel if communication errors occur during the attempt to disable
    if (!torpedo_marker_enabled || !actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (torpedo_marker_moving) {
        *errMsgOut = "Busy";
        return false;
    }

    if (torpedo_marker_hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (!torpedo_marker_homed) {
        *errMsgOut = "Not Homed";
        return false;
    }

    // Everything checks out, begin movement
    torpedo_marker_target_position = target_position;
    torpedo_marker_move_timeout = make_timeout_time_ms(MAX_MOVEMENT_TIME_MS);
    __compiler_memory_barrier();    // Make sure moving is set after we configure the parameters right, or else things might get squirly
    torpedo_marker_moving = true;
    dynamixel_set_target_position(torpedo_marker_id, target_position);

    return true;
}

void torpedo_marker_report_state(bool torque_enabled, bool moving, int32_t target_position,
                                    int32_t current_position, uint8_t hardware_err_status) {
    torpedo_marker_enabled = torque_enabled;
    torpedo_marker_hardware_err = (hardware_err_status != 0);

    if (torpedo_marker_moving) {
        // Timeout if we've been moving for too long
        if (time_reached(torpedo_marker_move_timeout)) {
            dynamixel_enable_torque(torpedo_marker_id, false);
            torpedo_marker_moving = false;
            torpedo_marker_enabled = false;
            LOG_WARN("Failed to reach target within timeout period");
            safety_raise_fault(FAULT_ACTUATOR_FAILURE);
        }

        // Make sure we're still enabled and the target is our target before processing the state data
        if (torque_enabled && target_position == torpedo_marker_target_position) {
            bool in_position;
            if (current_position > target_position + HOME_MARGIN) {
                in_position = false;
            }
            else if (current_position < target_position - HOME_MARGIN) {
                in_position = false;
            }
            else {
                in_position = true;
            }

            // If the servo has reached its destination, then we have arrived
            if (in_position && !moving) {
                // If we're going home, then mark moving as false, and allow new commands
                if (target_position == POSITION_HOME) {
                    torpedo_marker_moving = false;
                }
                // If not, we're going to a target, and we've got there. Time to go home
                else {
                    torpedo_marker_target_position = POSITION_HOME;
                    dynamixel_set_target_position(torpedo_marker_id, POSITION_HOME);
                }
            }
        }
    }
    else {
        // If we're not moving, make sure we're in the home position
        if (current_position > POSITION_HOME + HOME_MARGIN) {
            torpedo_marker_homed = false;
        }
        else if (current_position < POSITION_HOME - HOME_MARGIN) {
            torpedo_marker_homed = false;
        }
        else {
            torpedo_marker_homed = true;
        }
    }
}

// ========================================
// Internal Public Functions
// ========================================

void torpedo_marker_initialize(dynamixel_id id) {
    torpedo_marker_id = id;
}

void torpedo_marker_report_connect(void) {
    // TODO: Set mode to position control, with the limits that we want

    torpedo_marker_connected = true;
}

void torpedo_marker_report_disconnect(void) {
    // Clear state
    if (torpedo_marker_enabled || torpedo_marker_moving) {
        // Raise fault if a disconnect occurred while we were in an active state
        // This is important as a disconnect will disarm the dynamixel on re-connect
        // Also we can't trust what state the dynamixel will be in when it reconnects
        // It's better to start fresh, and fault that a previously sent command will fail
        safety_raise_fault(FAULT_ACTUATOR_FAILURE);
    }
    torpedo_marker_moving = false;
    torpedo_marker_enabled = false;
    torpedo_marker_connected = false;
}

bool torpedo_marker_arm(const char **errMsgOut) {
    if (!torpedo_marker_connected) {
        *errMsgOut = "Actuator Not Connected";
        return false;
    }

    dynamixel_enable_torque(torpedo_marker_id, true);

    // Reset fired status on re-arm
    // It's better to think we fired when we forget to reload the torpedos, then to forget a button
    // to manually re-reload during a competition run and the code doesn't let you fire the torpedo/markers
    torpedo_next_index = 0;
    dropper_next_index = 0;

    torpedo_marker_enabled = true;

    return true;
}

void torpedo_marker_safety_disable(void) {
    // Only send disable if we're enabled
    // Since we're constantly updating this from the RAM, if for whatever reason the disable torque fails
    // it will try again on the next kill switch update
    // Also we won't need to worry about initialization checks since enabled is always false when not initialized
    if (torpedo_marker_enabled) {
        // No need to check if connected, as torpedo_marker_enabled is cleared on disconnect
        dynamixel_enable_torque(torpedo_marker_id, false);

        // Now this *could* be non-deterministic behavior if torpedo_marker_safety_disable is called from a higher
        // interrupt priority than the async uart receive IRQ priority, as the refresh can't be aborted after the
        // refresh event callback fires. But if this preempts it, but before torpedo_marker_report_state sets
        // torpedo_marker_enabled, then it will be  overrwitten with old data from the refresh before torque is disabled.
        // But this firmware only gets kill switch updates from ROS, unlike the power board which uses a GPIO interrupt
        // so I'll just let it slide since solving this is a big pain and makes this whole code even more convoluted
        // then it already is.
        torpedo_marker_enabled = false;
    }
}

bool torpedo_marker_get_busy(void) {
    return torpedo_marker_moving;
}

bool torpedo_marker_set_home(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (torpedo_marker_hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (torpedo_marker_enabled || actuators_armed) {
        *errMsgOut = "Must be disarmed";
        return false;
    }

    struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(torpedo_marker_id);
    volatile struct dynamixel_ram *ram = dynamixel_get_ram(torpedo_marker_id);
    if (!eeprom || !ram) {
        *errMsgOut = "State read error";
        return false;
    }

    int32_t homing_offset_compensation = eeprom->homing_offset;
    if (homing_offset_compensation > 1024 || homing_offset_compensation < -1024) {
        homing_offset_compensation = 0;
    }

    int32_t new_homing_offset = (((homing_offset_compensation + ram->present_position) - POSITION_HOME) % 4096);
    if (new_homing_offset > 1024 || new_homing_offset < -1024) {
        *errMsgOut = "Homing offset too large";
        return false;
    }

    dynamixel_set_homing_offset(torpedo_marker_id, new_homing_offset);
    dynamixel_request_eeprom_rescan(torpedo_marker_id);

    LOG_INFO("Seting home to current position");

    return true;
}

bool torpedo_marker_move_home(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (!torpedo_marker_enabled || !actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (torpedo_marker_moving) {
        *errMsgOut = "Busy";
        return false;
    }

    if (torpedo_marker_hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    dynamixel_set_target_position(torpedo_marker_id, POSITION_HOME);
    LOG_INFO("Manually moving torpedo home");

    return true;
}

// ========================================
// Marker Dropper
// ========================================

uint8_t dropper_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_connected || !torpedo_marker_homed || torpedo_marker_hardware_err) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
    }
    else if (!torpedo_marker_enabled || !actuators_armed) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DISARMED;
    }
    else if (torpedo_marker_moving) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPING;
    }
    else if (dropper_next_index == 2) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPED;
    }
    else {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_READY;
    }
}

uint8_t dropper_get_available(void) {
    int avail = 2 - dropper_next_index;
    return (avail > 0 ? avail : 0);
}

bool dropper_drop_marker(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    int32_t target_position;

    if (dropper_next_index == 0) {
        target_position = POSITION_DROPPER1_FIRE;
    }
    else if (dropper_next_index == 1) {
        target_position = POSITION_DROPPER2_FIRE;
    }
    else {
        *errMsgOut = "All Dropped";
        return false;
    }

    if (torpedo_marker_set_target(errMsgOut, target_position)) {
        dropper_next_index++;
        LOG_INFO("Dropping Marker");
        return true;
    }
    else {
        return false;
    }
}

bool dropper_notify_reload(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (torpedo_marker_moving) {
        *errMsgOut = "Busy";
        return false;
    }

    dropper_next_index = 0;
    LOG_INFO("Marking dropper reloaded");
    return true;
}

// ========================================
// Torpedo
// ========================================

uint8_t torpedo_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_connected || !torpedo_marker_homed) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
    }
    else if (!torpedo_marker_enabled) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_DISARMED;
    }
    else if (torpedo_marker_moving) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRING;
    }
    else if (torpedo_next_index == 2) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRED;
    }
    else {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_CHARGED;
    }
}

uint8_t torpedo_get_available(void) {
    int avail = 2 - torpedo_next_index;
    return (avail > 0 ? avail : 0);
}

bool torpedo_fire(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    int32_t target_position;

    if (torpedo_next_index == 0) {
        target_position = POSITION_TORPEDO1_FIRE;
    }
    else if (torpedo_next_index == 1) {
        target_position = POSITION_TORPEDO2_FIRE;
    }
    else {
        *errMsgOut = "All Fired";
        return false;
    }

    if (torpedo_marker_set_target(errMsgOut, target_position)) {
        torpedo_next_index++;
        LOG_INFO("Firing Torpedo");
        return true;
    }
    else {
        return false;
    }
}

bool torpedo_notify_reload(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (torpedo_marker_moving) {
        *errMsgOut = "Busy";
        return false;
    }

    torpedo_next_index = 0;

    LOG_INFO("Marking torpedos reloaded");
    return true;
}
