#include "pico/binary_info.h"
#include "pico/sync.h"
#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"
#include "basic_logger/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuators"

// Global exports
bool actuators_initialized = false;
volatile bool actuators_armed = false;

// Public Functions
void actuators_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);
    bi_decl_if_func_used(bi_program_feature("Actuators V1"));

    torpedo_initialize();
    dropper_initialize();
    claw_initialize();

    actuators_initialized = true;
}

bool actuators_arm(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Make sure a kill switch interrupt won't fire in between checking conditions and arming
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Don't allow arming if already armed
    if (actuators_armed) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Already Armed";
        return false;
    }

    // Don't allow arming if killed
    if (safety_kill_get_asserting_kill()) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Kill Switch Removed";
        return false;
    }

    // We're good to arm
    actuators_armed = true;
    restore_interrupts(prev_interrupts);

    LOG_INFO("Arming Actuators");

    // Perform individual arm actions for actuators
    if (!torpedo_arm(errMsgOut)) {
        return false;
    }

    if (!dropper_notify_reload(errMsgOut)) {
        return false;
    }

    return true;
}

bool actuators_get_busy(void) {
    return torpedo_get_busy() || dropper_get_busy() || claw_get_busy();
}

void actuators_disarm(void) {
    actuators_armed = false;
    dropper_safety_disable();
    torpedo_safety_disable();
    claw_safety_disable();
}
