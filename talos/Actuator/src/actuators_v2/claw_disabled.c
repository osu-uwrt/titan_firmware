#include "actuators.h"
#include "actuators_internal.h"

enum claw_state claw_get_state(void) {
    return CLAW_STATE_UNINITIALIZED;
}

bool claw_open(const char **errMsgOut) {
    *errMsgOut = "Not Supported";
    return false;
}

bool claw_close(const char **errMsgOut) {
    *errMsgOut = "Not Supported";
    return false;
}
