#include "actuators.h"
#include "actuators_internal.h"

uint8_t claw_get_state(void) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    return riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
}

bool claw_open(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    *errMsgOut = "Not Supported";
    return false;
}

bool claw_close(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    *errMsgOut = "Not Supported";
    return false;
}
