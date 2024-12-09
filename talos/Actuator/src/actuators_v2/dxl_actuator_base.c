#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#include "titan/logger.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dxlact_base"

// ========================================
// Movement Management
// ========================================

bool dxlact_base_set_target(dxlact_state_t *state, const char **errMsgOut, int32_t target_position) {
    if (!state->connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    // Check both, as enabled reflects the actual dynamixel state, which might be cleared if an error occurs on the
    // dynamixel, while actuators_armed reflects if safety is permitting actuator movement, which might not be reflected
    // on the dynamixel if communication errors occur during the attempt to disable
    if (!state->enabled || !actuators_armed) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (state->move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    if (state->hardware_err) {
        *errMsgOut = "Hardware Error";
        return false;
    }

    if (!state->homed) {
        *errMsgOut = "Not Homed";
        return false;
    }

    // Everything checks out, begin movement
    state->target_position = target_position;
    state->move_timeout = make_timeout_time_ms(state->max_move_time_ms);
    __compiler_memory_barrier();  // Make sure moving is set after we configure the parameters right, or else things
                                  // might get squirly
    state->move_active = true;
    dynamixel_set_target_position(state->id, target_position);

    return true;
}

void dxlact_base_report_state(dxlact_state_t *state, bool torque_enabled, bool moving, int32_t target_position,
                              int32_t current_position, uint8_t hardware_err_status) {
    state->enabled = torque_enabled;
    state->hardware_err = (hardware_err_status != 0);

    if (state->move_active) {
        // Timeout if we've been moving for too long
        if (time_reached(state->move_timeout)) {
            state->move_active = false;
            actuators_disarm();
            LOG_WARN("DXL %d failed to reach target within timeout period", state->id);
            safety_raise_fault_with_arg(FAULT_ACTUATOR_FAILURE, state->id);
        }

        // Make sure we're still enabled and the target is our target before processing the state data
        else if (torque_enabled && target_position == state->target_position) {
            bool in_position;
            if (current_position > target_position + TARGET_POSITION_MARGIN) {
                in_position = false;
            }
            else if (current_position < target_position - TARGET_POSITION_MARGIN) {
                in_position = false;
            }
            else {
                in_position = true;
            }

            // If the servo has reached its destination, then we have arrived
            if (in_position && !moving) {
                // Call done handler and see if they want to continue or finish the move
                int32_t next_target;
                if (state->done_handler(state, &next_target)) {
                    state->target_position = next_target;
                    dynamixel_set_target_position(state->id, next_target);
                }
                else {
                    state->move_active = false;
                }
            }
        }
    }
    else {
        state->idle_handler(state, current_position);
    }
}

// ========================================
// Internal Public Functions
// ========================================

void dxlact_base_initialize(dxlact_state_t *state, dynamixel_id id, dxlact_idle_position_handler_t idle_handler,
                            dxlact_move_done_handler_t done_handler, uint32_t max_move_time_ms) {
    // Copy in provided init data
    state->id = id;
    state->max_move_time_ms = max_move_time_ms;
    state->idle_handler = idle_handler;
    state->done_handler = done_handler;

    // Clear state
    state->connected = false;
    state->move_active = false;
    state->enabled = false;
    state->homed = false;
    state->hardware_err = false;
}

void dxlact_base_report_connect(dxlact_state_t *state) {
    state->connected = true;
}

void dxlact_base_report_disconnect(dxlact_state_t *state) {
    // Clear state
    if (state->enabled || state->move_active) {
        // Raise fault if a disconnect occurred while we were in an active state
        // This is important as a disconnect will disarm the dynamixel on re-connect
        // Also we can't trust what state the dynamixel will be in when it reconnects
        // It's better to start fresh, and fault that a previously sent command will fail
        safety_raise_fault_with_arg(FAULT_ACTUATOR_FAILURE, state->id);
    }
    state->move_active = false;
    state->enabled = false;
    state->connected = false;
}

bool dxlact_base_arm(dxlact_state_t *state, const char **errMsgOut) {
    if (!state->connected) {
        *errMsgOut = "Actuator Not Connected";
        return false;
    }

    dynamixel_enable_torque(state->id, true);

    state->enabled = true;

    return true;
}

void dxlact_base_safety_disable(dxlact_state_t *state) {
    // Only send disable if we're enabled
    // Since we're constantly updating this from the RAM, if for whatever reason the disable torque fails
    // it will try again on the next kill switch update
    // Also we won't need to worry about initialization checks since enabled is always false when not initialized
    if (state->enabled) {
        // No need to check if connected, as torpedo_marker_enabled is cleared on disconnect
        dynamixel_enable_torque(state->id, false);

        // Now this *could* be non-deterministic behavior if torpedo_marker_safety_disable is called from a higher
        // interrupt priority than the async uart receive IRQ priority, as the refresh can't be aborted after the
        // refresh event callback fires. But if this preempts it, but before torpedo_marker_report_state sets
        // torpedo_marker_enabled, then it will be  overrwitten with old data from the refresh before torque is
        // disabled. But this firmware only gets kill switch updates from ROS, unlike the power board which uses a GPIO
        // interrupt so I'll just let it slide since solving this is a big pain and makes this whole code even more
        // convoluted then it already is.
        state->enabled = false;
    }
}

bool dxlact_base_get_busy(dxlact_state_t *state) {
    return state->move_active;
}
