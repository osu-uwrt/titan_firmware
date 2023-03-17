#include <stdbool.h>

#include "pico/assert.h"
#include "pico/time.h"
#include "hardware/watchdog.h"

#include "safety/profiler.h"
#include "safety_internal.h"


// ========================================
// Safety Limetime Functions
// ========================================

bool safety_initialized = false;
bool safety_is_setup = false;

static absolute_time_t watchdog_timeout_time;

void safety_setup(void) {
    hard_assert_if(SAFETY, safety_is_setup || safety_initialized);

    safety_internal_crash_reporting_handle_reset();
    safety_interface_setup();

    safety_is_setup = true;
    watchdog_timeout_time = make_timeout_time_ms(SAFETY_WATCHDOG_SETUP_TIMER_MS);
    watchdog_enable(SAFETY_WATCHDOG_SETUP_TIMER_MS, SAFETY_PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_init(void) {
    hard_assert_if(SAFETY, !safety_is_setup || safety_initialized);

    safety_internal_crash_reporting_handle_init();
    safety_internal_kill_handle_init();
    safety_interface_init();

    safety_initialized = true;
    watchdog_timeout_time = make_timeout_time_ms(SAFETY_WATCHDOG_ACTIVE_TIMER_MS);
    watchdog_enable(SAFETY_WATCHDOG_ACTIVE_TIMER_MS, SAFETY_PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_deinit(void) {
    hard_assert_if(SAFETY, !safety_is_setup || !safety_initialized);

    safety_internal_kill_handle_deinit();
    safety_interface_deinit();
    safety_internal_crash_reporting_handle_deinit();

    safety_initialized = false;
    watchdog_timeout_time = make_timeout_time_ms(SAFETY_WATCHDOG_SETUP_TIMER_MS);
    watchdog_enable(SAFETY_WATCHDOG_SETUP_TIMER_MS, SAFETY_PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_tick(void) {
    hard_assert_if(SAFETY, !safety_is_setup);

    // Check for any kill switch timeouts
    if (safety_initialized) {
        safety_internal_kill_refresh_switches();
    }

    safety_internal_fault_tick();
    safety_interface_tick();
    profiler_reset(true);

    if (absolute_time_diff_us(get_absolute_time(), watchdog_timeout_time) <
            1000 * (safety_initialized ? SAFETY_WATCHDOG_ACTIVE_FAULT_LESS_THAN_MS :
                    SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS)) {
        safety_raise_fault(FAULT_WATCHDOG_WARNING);
    }

    watchdog_timeout_time = make_timeout_time_ms(safety_initialized ? SAFETY_WATCHDOG_ACTIVE_TIMER_MS :
                                                 SAFETY_WATCHDOG_SETUP_TIMER_MS);
    watchdog_update();
}