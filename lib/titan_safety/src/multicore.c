#include "pico/multicore.h"

#include "safety_internal.h"

#include "pico/mutex.h"

/**
 * @brief Mutex to protect 64-bit value safety_multicore_next_checkin
 */
auto_init_mutex(safety_multicore_checkin_mtx);

/**
 * @brief The time which core 1 must check in to core 0, or else it will panic
 *
 */
static absolute_time_t safety_multicore_next_checkin = { 0 };

/**
 * @brief Boolean if core 1 is running, this determines if the check in will be checked during safety ticks
 *
 * This is exported so that it can be viewed by safety helper
 */
bool safety_multicore_running = false;

void safety_launch_core1(void (*entry)(void)) {
    assert(!safety_multicore_running);

    multicore_reset_core1();
    safety_multicore_next_checkin = make_timeout_time_ms(SAFETY_CORE1_CHECKIN_INTERVAL_MS);
    safety_multicore_running = true;
    multicore_launch_core1(entry);
}

void safety_core1_checkin(void) {
    assert(get_core_num() == 1);
    assert(safety_multicore_running);

    mutex_enter_blocking(&safety_multicore_checkin_mtx);
    // Note the mutex is required because we're writing a 64-bit value (not atomic)
    safety_multicore_next_checkin = make_timeout_time_ms(SAFETY_CORE1_CHECKIN_INTERVAL_MS);
    mutex_exit(&safety_multicore_checkin_mtx);
}

void safety_internal_multicore_tick(void) {
    if (safety_multicore_running) {
        mutex_enter_blocking(&safety_multicore_checkin_mtx);
        // Note the mutex is required because we're reading a 64-bit value (not atomic)
        bool alive = !time_reached(safety_multicore_next_checkin);
        mutex_exit(&safety_multicore_checkin_mtx);

        if (!alive) {
            // Set the nmi variable so that when panic sends an NMI to halt core 1, it'll save the last instruction addr
            core1_nmi_capture_addr = true;

            panic("Core 1 did not check in to safety within required interval");
        }
    }
}
