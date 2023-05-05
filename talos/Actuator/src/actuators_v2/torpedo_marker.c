#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"
#include "basic_logger/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "torpedo_marker"

static bool torpedo_marker_moving = false;
static bool torpedo_marker_enabled = false;
static dynamixel_id torpedo_marker_id;

void torpedo_marker_initialize(dynamixel_id id) {
    torpedo_marker_id = id;
}

bool torpedo_marker_arm(const char **errMsgOut) {
    if (!dynamixel_check_connected(torpedo_marker_id)) {
        *errMsgOut = "Actuator Not Connected";
        return false;
    }

    dynamixel_enable_torque(torpedo_marker_id, true);

    torpedo_marker_enabled = true;
    return true;
}

void torpedo_marker_safety_disable(void) {
    if (torpedo_marker_enabled) {
        if (!dynamixel_check_connected(torpedo_marker_id)) {
            LOG_ERROR("Lost Connection Before Disable!");
            safety_raise_fault(FAULT_ACTUATOR_FAILURE);
            // Don't return here, just let it fall through and attempt the torque disable command
        }

        dynamixel_enable_torque(torpedo_marker_id, false);
        torpedo_marker_enabled = false;
    }
}

enum dropper_state dropper_get_state(void) {
    if (!dynamixel_check_connected(torpedo_marker_id)) {
        return DROPPER_STATE_ERROR;
    }
    else if (!torpedo_marker_enabled) {
        return DROPPER_STATE_DISARMED;
    }
    else if (torpedo_marker_moving) {
        return DROPPER_STATE_DROPPING;
    }
    else {
        return DROPPER_STATE_READY;
    }
}

bool dropper_drop_marker(const char **errMsgOut) {
    if (!dynamixel_check_connected(torpedo_marker_id)) {
        *errMsgOut = "Actuator Not Connected";
        return false;
    }

    if (safety_kill_get_asserting_kill()) {
        *errMsgOut = "Kill Switch Removed";
        return false;
    }

    if (!actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (torpedo_marker_moving) {
        *errMsgOut = "Busy";
        return false;
    }


}

bool dropper_notify_reload(const char **errMsgOut) {

}

enum torpedo_state torpedo_get_state(void) {

}

bool torpedo_fire(const char **errMsgOut) {

}

bool torpedo_notify_reload(const char **errMsgOut) {

}
