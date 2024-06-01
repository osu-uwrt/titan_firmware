#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#include "titan/logger.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo_marker"

// Location Definitions
/**
 * @brief The servo position corresponding to the rest position of the servo
 */
#define POSITION_HOME 2048

/**
 * @brief Positions for various fire targets
 */
#define POSITION_DROPPER1_FIRE 3072
#define POSITION_DROPPER2_FIRE 4076
#define POSITION_TORPEDO1_FIRE 1024
#define POSITION_TORPEDO2_FIRE 5

#define MAX_MOVEMENT_TIME_MS 5000

// Torpedo Index tracking
// These variables are not safe to modify in interrupts
static dxlact_state_t *torpedo_marker_state;
static uint torpedo_next_index = 0;
static uint dropper_next_index = 0;

// ========================================
// Dynamixel Actuator Base Callbacks
// ========================================

static void torpedo_marker_idle_handler(dxlact_state_t *state, int32_t current_position) {
    // If we're not moving, make sure we're in the home position
    if (current_position > POSITION_HOME + TARGET_POSITION_MARGIN) {
        state->homed = false;
    }
    else if (current_position < POSITION_HOME - TARGET_POSITION_MARGIN) {
        state->homed = false;
    }
    else {
        state->homed = true;
    }
}

static bool torpedo_marker_move_done_handler(dxlact_state_t *state, int32_t *next_target) {
    // If we're going home, return false and finish the move
    if (state->target_position == POSITION_HOME) {
        return false;
    }
    // If not, we're going to a target, and we've got there. Time to go home
    else {
        *next_target = POSITION_HOME;
        return true;
    }
}

// ========================================
// Internal Public Functions
// ========================================

void torpedo_marker_initialize(dxlact_state_t *state, dynamixel_id id) {
    torpedo_marker_state = state;
    dxlact_base_initialize(state, id, &torpedo_marker_idle_handler, &torpedo_marker_move_done_handler,
                           MAX_MOVEMENT_TIME_MS);
}

void torpedo_marker_reset_count(void) {
    torpedo_next_index = 0;
    dropper_next_index = 0;
}

bool torpedo_marker_set_home(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_state->connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (torpedo_marker_state->hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (torpedo_marker_state->enabled || actuators_armed) {
        *errMsgOut = "Must be disarmed";
        return false;
    }

    struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(torpedo_marker_state->id);
    volatile struct dynamixel_ram *ram = dynamixel_get_ram(torpedo_marker_state->id);
    if (!eeprom || !ram) {
        *errMsgOut = "State read error";
        return false;
    }

    int32_t homing_offset_compensation = eeprom->homing_offset;
    if (homing_offset_compensation > 1024 || homing_offset_compensation < -1024) {
        homing_offset_compensation = 0;
    }

    int32_t new_homing_offset = -1 * ((ram->present_position - homing_offset_compensation) - POSITION_HOME) % 4096;
    if (new_homing_offset > 1024 || new_homing_offset < -1024) {
        *errMsgOut = "Homing offset too large";
        return false;
    }

    dynamixel_set_homing_offset(torpedo_marker_state->id, new_homing_offset);
    dynamixel_request_eeprom_rescan(torpedo_marker_state->id);

    LOG_INFO("Seting home to current position");

    return true;
}

bool torpedo_marker_move_home(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_state->connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (torpedo_marker_state->hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (!torpedo_marker_state->enabled || !actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (torpedo_marker_state->move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    dynamixel_set_target_position(torpedo_marker_state->id, POSITION_HOME);
    LOG_INFO("Manually moving torpedo home");

    return true;
}

// ========================================
// Marker Dropper
// ========================================

uint8_t dropper_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!torpedo_marker_state->connected || !torpedo_marker_state->homed || torpedo_marker_state->hardware_err) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
    }
    else if (!torpedo_marker_state->enabled || !actuators_armed) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DISARMED;
    }
    else if (torpedo_marker_state->move_active) {
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

    if (dxlact_base_set_target(torpedo_marker_state, errMsgOut, target_position)) {
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

    if (torpedo_marker_state->move_active) {
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

    if (!torpedo_marker_state->connected || !torpedo_marker_state->homed || torpedo_marker_state->hardware_err) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
    }
    else if (!torpedo_marker_state->enabled) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_DISARMED;
    }
    else if (torpedo_marker_state->move_active) {
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

    if (dxlact_base_set_target(torpedo_marker_state, errMsgOut, target_position)) {
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

    if (torpedo_marker_state->move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    torpedo_next_index = 0;

    LOG_INFO("Marking torpedos reloaded");
    return true;
}
