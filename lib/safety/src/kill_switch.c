#include <stdbool.h>
#include <stdint.h>

#include "pico/assert.h"
#include "pico/time.h"

#include "safety_internal.h"

static absolute_time_t last_kill_switch_change;
static bool last_state_asserting_kill = true; // Start asserting kill

/**
 * @brief Local utility function to do common tasks for when the system is killed
 * Called from any function that needs to kill the system
 *
 * This function should only be called when safety is initialized
 */
static void safety_local_handle_kill(void) {
    last_state_asserting_kill = true;
    last_kill_switch_change = get_absolute_time();

    safety_handle_kill();

    LOG_DEBUG("Kill Switch Asserting Kill");
}


void safety_internal_kill_refresh_switches(void) {
    // Check all kill switches for asserting kill
    bool asserting_kill = false;
    int num_switches_enabled = 0;
    for (int i = 0; i < num_kill_switches; i++) {
        if (kill_switch_states[i].enabled) {
            num_switches_enabled++;

            // Kill if asserting kill or if timeout expired when requiring update
            if (kill_switch_states[i].asserting_kill ||
                    (kill_switch_states[i].needs_update && time_reached(kill_switch_states[i].update_timeout))) {
                asserting_kill = true;
                break;
            }
        }
    }

    // If no kill switches are enabled, force into kill as a precaution
    if (num_switches_enabled == 0) {
        asserting_kill = true;
    }

    // Update last state, and notify of kill if needed
    if (last_state_asserting_kill != asserting_kill) {
        last_state_asserting_kill = asserting_kill;

        if (asserting_kill) {
            safety_local_handle_kill();
        } else {
            LOG_DEBUG("Kill Switch Okay (Enabled)");
            last_kill_switch_change = get_absolute_time();
            safety_handle_enable();
        }
    }
}

void safety_internal_kill_handle_init(void) {
    // Populate the last kill switch change time to when safety is set up
    last_kill_switch_change = get_absolute_time();
}


void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update){
    valid_params_if(SAFETY, switch_num < num_kill_switches);

    kill_switch_states[switch_num].asserting_kill = asserting_kill;
    kill_switch_states[switch_num].update_timeout = make_timeout_time_ms(SAFETY_KILL_SWITCH_TIMEOUT_MS);
    kill_switch_states[switch_num].needs_update = needs_update;
    kill_switch_states[switch_num].enabled = true;

    if (safety_initialized && asserting_kill) {
        safety_local_handle_kill();
    }
}

bool safety_kill_get_asserting_kill(void) {
    hard_assert_if(SAFETY, !safety_initialized);
    return last_state_asserting_kill;
}

absolute_time_t safety_kill_get_last_change(void) {
    hard_assert_if(SAFETY, !safety_initialized);
    return last_kill_switch_change;
}