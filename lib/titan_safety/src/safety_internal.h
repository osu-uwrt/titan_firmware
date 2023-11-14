#ifndef SAFETY_INTERNAL_H_
#define SAFETY_INTERNAL_H_

#include "titan/logger.h"
#include "titan/safety.h"

#include <stdbool.h>
#include <stdint.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "safety"

/**
 * @file safety_internal.h
 *
 * Internal API for the safety library code.
 */

// ========================================
// Configuration Functions
// ========================================

// PICO_CONFIG: SAFETY_WATCHDOG_ACTIVE_TIMER_MS, Watchdog timer duration when safety is setup but not initialized in milliseconds. Useful for long-running initialization code, type=int, default=3000, group=titan_safety
#ifndef SAFETY_WATCHDOG_SETUP_TIMER_MS
#define SAFETY_WATCHDOG_SETUP_TIMER_MS 3000
#endif

// PICO_CONFIG: SAFETY_WATCHDOG_ACTIVE_TIMER_MS, Watchdog timer duration when safety is initialized in milliseconds, type=int, default=250, group=titan_safety
#ifndef SAFETY_WATCHDOG_ACTIVE_TIMER_MS
#define SAFETY_WATCHDOG_ACTIVE_TIMER_MS 250
#endif

// PICO_CONFIG: SAFETY_PAUSE_WATCHDOG_ON_DEBUG, Allows watchdog timer to pause when CPU is being debugged. Required to be 1 during debugging, type=bool, default=0, group=titan_safety
#ifndef SAFETY_PAUSE_WATCHDOG_ON_DEBUG
#define SAFETY_PAUSE_WATCHDOG_ON_DEBUG 1
#endif

// PICO_CONFIG: SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS, Remaining time before watchdog reset when a fault should be raised warning of close to reset when safety is setup but not initialized in milliseconds. Useful for long-running initialization code, type=int, default=500, group=titan_safety
#ifndef SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS
#define SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS 500
#endif

// PICO_CONFIG: SAFETY_WATCHDOG_SETUP_FAULT_LESS_THAN_MS, Remaining time before watchdog reset when a fault should be raised warning of close to reset when safety initialized in milliseconds. Useful for long-running initialization code, type=int, default=100, group=titan_safety
#ifndef SAFETY_WATCHDOG_ACTIVE_FAULT_LESS_THAN_MS
#define SAFETY_WATCHDOG_ACTIVE_FAULT_LESS_THAN_MS 160
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_SAFETY, Enable/disable assertions for safety library, type=bool, default=0, group=titan_safety
#ifndef PARAM_ASSERTIONS_ENABLED_SAFETY
#define PARAM_ASSERTIONS_ENABLED_SAFETY 0
#endif

// ========================================
// External Interface Functions
// ========================================
// NOTE: These functions need to be defined in the code using the safety library

extern const int num_kill_switches;
extern struct kill_switch_state kill_switch_states[];

/**
 * @brief Called to set the fault led light
 *
 * @note This function is called during safety tick
 *
 * @param on Logic level of the light
 */
void safety_set_fault_led(bool on);

/**
 * @brief Handles a kill request from a kill switch
 *
 * @attention This function may be called during an interrupt
 */
void safety_handle_kill(void);

/**
 * @brief Called when all kill switches are no longer asserting a kill and the system is re-enabled
 *
 * @note This function is called during safety tick
 */
void safety_handle_enable(void);

/**
 * @brief Looks up fault id name for a given fault id
 *
 * @param fault_id The fault id to lookup
 * @return const char* The fault name
 */
const char *safety_lookup_fault_id(uint32_t fault_id);

/**
 * @brief Called for implementation specific code during safety_setup
 */
void safety_interface_setup(void);

/**
 * @brief Called for implementation specific code during safety_init
 */
void safety_interface_init(void);

/**
 * @brief Called for implementation specific code during safety_deinit
 */
void safety_interface_deinit(void);

/**
 * @brief Called for implementation specific code during safety_tick
 *
 * Put any code that must be called (else the watchdog will timeout) in here
 */
void safety_interface_tick(void);

// ========================================
// Internal Functions
// ========================================

/**
 * @brief Function to initialize crash reporting after device reset
 */
void safety_internal_crash_reporting_handle_reset(void);

/**
 * @brief Notifies crash reporting that safety has been initialized
 */
void safety_internal_crash_reporting_handle_init(void);

/**
 * @brief Notifies kill switch management that safety has been initialized
 */
void safety_internal_kill_handle_init(void);

/**
 * @brief Performs core initialization required for the safety fault system.
 *
 * @attention Must be called before raising any faults
 */
void safety_internal_fault_setup(void);

/**
 * @brief Ticks fault reporting
 */
void safety_internal_fault_tick(void);

/**
 * @brief Notifies crash reporting that safety has been deinitialized
 */
void safety_internal_crash_reporting_handle_deinit(void);

/**
 * @brief Notifies kill switch management that safety has been deinitialized
 */
void safety_internal_kill_handle_deinit(void);

/**
 * @brief Refreshes kill switches to check for any timeouts
 * It is responsible for re-enabling the robot after all of the kill switches have been released.
 *
 * This function should only be called when safety is initialized
 */
void safety_internal_kill_refresh_switches(void);

#endif
