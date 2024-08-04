#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#include "titan/logger.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "claw"

#define CLAW_OPEN_POSITION 12288
#define CLAW_CLOSE_POSITION 6144
// Minimum position, below this point we consider home is lost (it shouldn't be possible for the claw to go below this)
#define CLAW_MIN_POSITION 4608
static_assert(CLAW_MIN_POSITION >= 4096, "Claw min position must be greater than revolution");

#define MAX_CLAW_MOVE_TIME_MS 10000

enum claw_position { CLAW_OPENED, CLAW_CLOSED, CLAW_MIDDLE };
static dxlact_state_t *claw_state;
static enum claw_position claw_current_position;  // Only valid if claw_state->homed true

// ========================================
// Dynamixel Actuator Base Callbacks
// ========================================

static void claw_idle_handler(dxlact_state_t *state, int32_t current_position) {
    // Make sure our current position is valid for the claw
    if (current_position < CLAW_MIN_POSITION) {
        state->homed = false;
    }
    else {
        state->homed = true;
        if (current_position >= (CLAW_CLOSE_POSITION - TARGET_POSITION_MARGIN) &&
            current_position <= (CLAW_CLOSE_POSITION - TARGET_POSITION_MARGIN)) {
            claw_current_position = CLAW_CLOSED;
        }
        else if (current_position >= (CLAW_OPEN_POSITION - TARGET_POSITION_MARGIN) &&
                 current_position <= (CLAW_OPEN_POSITION - TARGET_POSITION_MARGIN)) {
            claw_current_position = CLAW_OPENED;
        }
        else {
            claw_current_position = CLAW_MIDDLE;
        }
    }
}

static bool claw_move_done_handler(dxlact_state_t *state, int32_t *next_target) {
    // After we stop moving we're always done (we don't have complex mvoements we need to do)
    (void) state;
    (void) next_target;
    return false;
}

// ========================================
// Internal Public Functions
// ========================================

void claw_initialize(dxlact_state_t *state, dynamixel_id id) {
    claw_state = state;
    dxlact_base_initialize(state, id, &claw_idle_handler, &claw_move_done_handler, MAX_CLAW_MOVE_TIME_MS);
}

bool claw_set_closed_position(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!claw_state->connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (claw_state->hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (claw_state->enabled || actuators_armed) {
        *errMsgOut = "Must be disarmed";
        return false;
    }

    struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(claw_state->id);
    volatile struct dynamixel_ram *ram = dynamixel_get_ram(claw_state->id);
    if (!eeprom || !ram) {
        *errMsgOut = "State read error";
        return false;
    }

    int32_t homing_offset_compensation = eeprom->homing_offset;
    int32_t new_homing_offset = -1 * ((ram->present_position - homing_offset_compensation) - CLAW_CLOSE_POSITION);
    // TODO: Make sure this weeks with stacking close compensation

    dynamixel_set_homing_offset(claw_state->id, new_homing_offset);
    dynamixel_request_eeprom_rescan(claw_state->id);

    LOG_INFO("Seting close position to current position");

    return true;
}

bool claw_creep_delta(const char **errMsgOut, int32_t move_delta) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!claw_state->connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    if (claw_state->hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (!claw_state->enabled || !actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (claw_state->move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    volatile struct dynamixel_ram *ram = dynamixel_get_ram(claw_state->id);
    if (!ram) {
        *errMsgOut = "State read error";
        return false;
    }

    dynamixel_set_target_position(claw_state->id, ram->present_position + move_delta);
    LOG_INFO("Manually moving claw by %ld ticks", move_delta);

    return true;
}

// ========================================
// Marker Dropper
// ========================================

uint8_t claw_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (!claw_state->connected || !claw_state->homed || claw_state->hardware_err) {
        return riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
    }
    else if (!claw_state->enabled || !actuators_armed) {
        return riptide_msgs2__msg__ActuatorStatus__CLAW_DISARMED;
    }
    else if (claw_state->move_active) {
        if (claw_state->target_position == CLAW_OPEN_POSITION) {
            return riptide_msgs2__msg__ActuatorStatus__CLAW_OPENING;
        }
        else if (claw_state->target_position == CLAW_CLOSE_POSITION) {
            return riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSING;
        }
        else {
            return riptide_msgs2__msg__ActuatorStatus__CLAW_UNKNOWN;
        }
    }
    else {
        switch (claw_current_position) {
        case CLAW_OPENED:
            return riptide_msgs2__msg__ActuatorStatus__CLAW_OPENED;
        case CLAW_CLOSED:
            return riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSED;
        default:
            return riptide_msgs2__msg__ActuatorStatus__CLAW_UNKNOWN;
        }
    }
}

bool claw_open(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (dxlact_base_set_target(claw_state, errMsgOut, CLAW_OPEN_POSITION)) {
        LOG_INFO("Opening Claw");
        return true;
    }
    return false;
}

bool claw_close(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    if (dxlact_base_set_target(claw_state, errMsgOut, CLAW_CLOSE_POSITION)) {
        LOG_INFO("Closing Claw");
        return true;
    }
    return false;
}
