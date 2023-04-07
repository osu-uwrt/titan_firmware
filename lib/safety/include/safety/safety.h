#ifndef _SAFETY__SAFETY_H
#define _SAFETY__SAFETY_H

#include <stdbool.h>
#include "pico/time.h"


/**
 * @file safety.h
 *
 * API for managing firmware safety/reliability related functions.
 *
 * This library has four main components
 *  - Fault Reporting: Implements a fault tracking system in a 32-bit ID. If a fault occurs, a bit will be set
 *      corresponding to a fault ID defined by the implementer in the fault list. A callback will be raised to indicate
 *      to the user that a fault has occurred (such as an LED lighting up).
 *
 *  - Kill Switch: Implements a set of functions to allow multiple kill switches to be assigned, which can be confiugred
 *      to require a constant update to remain enabled. Upon kill switch activation, an implementation defined callback
 *      allows the implementer to immediately respond to a kill switch request. Functions are also implemented to check
 *      the current kill switch state, and be notified when the system is enabled
 *
 *  - Crash Reporting: Hooks various parts of the low-level firmware to provide in-depth information about any crashes
 *      in the system to aid in debugging. This code hooks the systick handler to report an uptime, the hardfault,
 *      assertion, and panic handlers to provide crash information, and stores this information for SAFETY_NUM_CRASH_LOG_ENTRIES.
 *
 *  - Watchdog Timer: All of these functions are integrated with the chip's watchdog timer such that critical code,
 *      such as kill switch updates, are never missed
 */

// PICO_CONFIG: SAFETY_KILL_SWITCH_TIMEOUT_MS, Timeout in milliseconds of kill switch commands requiring updating. Note it may take longer to disable as it is only refreshed during safety_tick, type=int, default=500, group=safety
#ifndef SAFETY_KILL_SWITCH_TIMEOUT_MS
#define SAFETY_KILL_SWITCH_TIMEOUT_MS 500
#endif

// PICO_CONFIG: SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE, Maximum size of software kill frame string to avoid competing software kill publishers, type=str, default=32, group=safety
#ifndef SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE
#define SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE 32
#endif


// ========================================
// Fault Management Functions
// ========================================

// MAX_FAULT_ID is determined by the fault_list_reg size (32-bit)
#define MAX_FAULT_ID          31
#define FAULT_WATCHDOG_RESET   0
#define FAULT_WATCHDOG_WARNING 1
// All other fault ids are implentation specific

/**
 * @brief A pointer to the list of all the active faults as bits
 *
 * Do not write to this variable, use safety_raise_fault/safety_lower_fault instead
 */
extern volatile uint32_t * const fault_list_reg;

/**
 * @brief Raises the specified fault id
 *
 * This function can be used in interrupt callbacks
 *
 * @param fault_id The id to be raised. Faults are defined above
 */
void safety_raise_fault(uint32_t fault_id);

/**
 * @brief Lowers the specified fault id
 *
 * This function can be used in interrupt callbacks
 *
 * @param fault_id The id to be lowered. Faults are defined above
 */
void safety_lower_fault(uint32_t fault_id);



// ========================================
// Kill Switch Management Functions
// ========================================

struct kill_switch_state {
    bool enabled;                   // If the specific kill switch is enabled
    bool asserting_kill;            // If the kill switch is asserting a kill request (system disable)
    bool needs_update;              // If the switch needs to be updated or it will be considered killed
    absolute_time_t update_timeout; // The last update timestamp of the switch

    // The frame that asserted the kill switch
    // Prevents another node from de-asserting kill by publishing that it is not killed with the same switch id
    char locking_frame[SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE];
};

/**
 * @brief State of all of the kill switches for reading
 *
 * Do not write to this variable, use safety_kill_switch_update instead
 */
extern struct kill_switch_state kill_switch_states[];

/**
 * @brief Updates a kill switch state
 *
 * safety_init must be called before this function can be used
 * This function can be used in interrupt callbacks
 *
 * @param switch_num The unique number for that kill switch. MUST BE < MAX_KILL_SWITCHES
 * @param asserting_kill If the kill switch is asserting a kill request (system disable)
 * @param needs_update Setting to true will require the kill switch to be updated within SAFETY_KILL_SWITCH_TIMEOUT_MS or else will assert kill
 */
void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update);

/**
 * @brief Returns if the kill switch is asserting a safety kill
 *
 * safety_init must be called before this function can be used
 * This function can be used in interrupt callbacks
 *
 * @return true  Any action requiring a kill switch insertion should not run
 * @return false All kill switches are reporting okay and the operation can run
 */
bool safety_kill_get_asserting_kill(void);

/**
 * @brief Gets the time of the last kill switch change
 *
 * safety_init must be called before this function can be used
 *
 * @return absolute_time_t The time of last kill switch change as an absolute time
 */
absolute_time_t safety_kill_get_last_change(void);


// ========================================
// Safety Watchdog-Related Functions
// ========================================

/**
 * @brief Pointer to depth sensor calibration data which will persist watchdog resets.
 */
extern volatile uint32_t * const depth_cal_reg;

/**
 * @brief Value contained in depth_cal_reg if the register contains invalid data
 */
#define DEPTH_CALIBRATION_INVALID 0xFFFFFFFF

/**
 * @brief Notify safety that a software reset is ocurring.
 * This sets the required watchdog scratch registers to notify of a clean boot via a watchdog reset.
 * If this is not called, it will instead report the source of the clean boot as whatever the vreg peripherial's RESET_CAUSE is.
 */
void safety_notify_software_reset(void);

/**
 * @brief Resets into bootloader mode.
 * This issues a watchdog reset with the proper flags set to enter bootloader mode.
 * This function does not return
 */
void safety_enter_bootloader(void);

/**
 * @brief Prints the full watchdog crash log to the system log
 *
 * safety_setup must be called before this function can be used
 */
void safety_print_crash_log(void);

// ========================================
// Safety Limetime Functions
// ========================================

/**
 * @brief Boolean for if safety_setup has been called
 */
extern bool safety_is_setup;

/**
 * @brief Boolean for if safety_init has been called
 */
extern bool safety_initialized;

/**
 * @brief Performs core safety setup to be completed immediately after reset
 * This enables a watchdog timer, but with a multi-second long tick for setup
 * Will also print data on the last reset cause and any crash data from that
 *
 * Can only be called once
 */
void safety_setup(void);

/**
 * @brief Initializes safety for normal operation
 * This will tighten the timing for the watchdog timer, and is required to use the kill switch features
 *
 * safety_setup must be called before this function can be used
 */
void safety_init(void);

/**
 * @brief Deinitializes safety for setup
 * This will loosen the timing for the watchdog timer.
 *
 * safety_init must be called before this function can be used
 */
void safety_deinit(void);

/**
 * @brief Ticks safety
 * This must be called within the period of the watchdog timer or a reset will occur
 *
 * safety_setup must be called before this function can be used
 */
void safety_tick(void);


#endif