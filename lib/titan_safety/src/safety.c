#include "safety_internal.h"

#include "hardware/exception.h"
#include "hardware/regs/intctrl.h"
#include "hardware/structs/syscfg.h"
#include "hardware/watchdog.h"
#include "pico/assert.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include <stdbool.h>

// ========================================
// Safety Limetime Functions
// ========================================

static_assert(SAFETY_WATCHDOG_ALARM_NUM < NUM_TIMERS, "Invalid SAFETY_WATCHDOG_ALARM_NUM");

bool safety_initialized = false;
bool safety_is_setup = false;

static absolute_time_t watchdog_timeout_time;

void safety_setup(void) {
    hard_assert_if(SAFETY, safety_is_setup || safety_initialized);
    bi_decl_if_func_used(bi_program_feature("Titan Safety"));
    safety_internal_fault_setup();
    safety_internal_crash_reporting_handle_reset();
    safety_interface_setup();

    // Claim the alarm for watchdog timeout handling
#if SAFETY_WATCHDOG_ALARM_NUM >= 0
    hardware_alarm_claim(SAFETY_WATCHDOG_ALARM_NUM);

    // Disarm alarm in case it is running
    timer_hw->armed = (1 << SAFETY_WATCHDOG_ALARM_NUM);

    // Clear pending IRQ in timer (in case it fired for whatever reason)
    timer_hw->intr = 1u << SAFETY_WATCHDOG_ALARM_NUM;

    // Enable IRQ in timer hardware
    timer_hw->inte = 1u << SAFETY_WATCHDOG_ALARM_NUM;

    // Set the NMI handler to point to watchdog timeout handling
    exception_set_exclusive_handler(NMI_EXCEPTION, &safety_nmi_handler);

    // Enable NMI for the alarm
    assert(get_core_num() == 0);
    syscfg_hw->proc0_nmi_mask = 1 << (TIMER_IRQ_0 + SAFETY_WATCHDOG_ALARM_NUM);
#endif

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
    safety_internal_multicore_tick();
    profiler_reset(true);

    int64_t watchdog_time_remaining = absolute_time_diff_us(get_absolute_time(), watchdog_timeout_time);
    if (watchdog_time_remaining < 1000 * (safety_initialized ? SAFETY_WATCHDOG_ACTIVE_FAULT_LESS_THAN_MS :
                                                               SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS)) {
        safety_raise_fault_with_arg(FAULT_WATCHDOG_WARNING, watchdog_time_remaining);
    }

    watchdog_timeout_time =
        make_timeout_time_ms(safety_initialized ? SAFETY_WATCHDOG_ACTIVE_TIMER_MS : SAFETY_WATCHDOG_SETUP_TIMER_MS);

#if SAFETY_WATCHDOG_ALARM_NUM >= 0
    // Compute time, one tick before the watchdog fires
    // Because the watchdog and timer are tied to the same clock source, we can do this, and are gaurenteed to be okay
    // Note this could be between 1-2 us before the reboot, depending on when the next tick occurs
    uint32_t nmi_time = ((uint32_t) (to_us_since_boot(watchdog_timeout_time) & 0xFFFFFFFF)) - 1;
    // Schedule the alarm to fire right before the watchdog goes
    timer_hw->alarm[SAFETY_WATCHDOG_ALARM_NUM] = nmi_time;
#endif

    watchdog_update();
}
