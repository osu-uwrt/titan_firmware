#include <stdbool.h>

#include "pico/assert.h"
#include "hardware/watchdog.h"

#include "safety_internal.h"


// ========================================
// Safety Limetime Functions
// ========================================

bool safety_initialized = false;
bool safety_is_setup = false;

void safety_setup(void) {
    hard_assert_if(SAFETY, safety_is_setup || safety_initialized);

    safety_internal_crash_reporting_handle_reset();
    safety_interface_setup();

    safety_is_setup = true;
    watchdog_enable(SAFETY_WATCHDOG_SETUP_TIMER_MS, SAFETY_PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_init(void) {
    hard_assert_if(SAFETY, !safety_is_setup || safety_initialized);

    safety_internal_crash_reporting_handle_init();
    safety_internal_kill_handle_init();
    safety_interface_init();

    safety_initialized = true;
    watchdog_enable(SAFETY_WATCHDOG_ACTIVE_TIMER_MS, SAFETY_PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_tick(void) {
    hard_assert_if(SAFETY, !safety_is_setup);

    // Check for any kill switch timeouts
    if (safety_initialized) {
        safety_internal_kill_refresh_switches();
    }

    watchdog_update();
}