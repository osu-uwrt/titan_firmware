#ifndef _SAFETY__SAFETY_H
#define _SAFETY__SAFETY_H

#include <stdbool.h>
#include "pico/stdlib.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK, Enable/disable assertions for function lifetimes, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK
#define PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK 0
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_SAFETY, Enable/disable assertions for safety functions, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_SAFETY
#define PARAM_ASSERTIONS_ENABLED_SAFETY 0
#endif

// ========================================
// Fault Management Functions
// ========================================

#define MAX_FAULT_ID          31
#define FAULT_WATCHDOG_RESET   0
// All other fault ids are implentation specific

/**
 * @brief A pointer to the list of all the active faults as bits
 * DO NOT WRITE TO THIS! There are methods to safely raise and lower faults
 */
extern volatile uint32_t * const fault_list;

/**
 * @brief Raises the specified fault id
 * 
 * INTERRUPT SAFE
 * 
 * @param fault_id The id to be raised. Faults are defined above
 */
void safety_raise_fault(uint32_t fault_id);

/**
 * @brief Lowers the specified fault id
 * 
 * INTERRUPT SAFE
 * 
 * @param fault_id The id to be lowered. Faults are defined above
 */
void safety_lower_fault(uint32_t fault_id);



// ========================================
// Kill Switch Management Functions
// ========================================

#define KILL_SWITCH_TIMEOUT_MS 500
#define SOFTWARE_KILL_FRAME_STR_SIZE 32
struct kill_switch_state {
    bool enabled;                   // If the specific kill switch is enabled
    bool asserting_kill;            // If the kill switch is asserting a robot kill
    bool needs_update;              // If the switch needs to be updated or it will be considered killed
    absolute_time_t update_timeout; // The last update timestamp of the switch

    // The frame that asserted the kill switch
    // Prevents another node from de-asserting kill by publishing that it is not killed with the same switch id
    char locking_frame[SOFTWARE_KILL_FRAME_STR_SIZE];
};

/**
 * @brief State of all of the kill switches
 * DO NOT WRITE TO THIS! Use methods to write to this variable instead
 */
extern struct kill_switch_state kill_switch_states[];

/**
 * @brief Updates a kill switch state
 * 
 * INTERRUPT SAFE
 * 
 * @param switch_num The unique number for that kill switch. MUST BE < MAX_KILL_SWITCHES 
 * @param asserting_kill If the kill switch is asserting a kill switch and the robot should be killed
 * @param needs_update Setting to true will require the kill switch to be updated within KILL_SWITCH_TIMEOUT_MS or else will assert kill
 */
void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update);

/**
 * @brief Returns if the kill switch is asserting a safety kill
 * 
 * INTERRUPT SAFE
 * REQUIRES INITIALIZATION
 * 
 * @return true  Any action requiring a kill switch should not run
 * @return false All kill switches are reporting okay and the operation can run
 */
bool safety_kill_get_asserting_kill(void);

/**
 * @brief Gets the time of the last kill switch change
 * 
 * INTERRUPT SAFE? (64-bit value internally)
 * REQUIRES INITIALIZATION
 * 
 * @return absolute_time_t The time of last kill switch change as an absolute time
 */
absolute_time_t safety_kill_get_last_change(void);


// ========================================
// Safety Limetime Functions
// ========================================

/**
 * @brief Boolean for if safety_setup has been called
 * If code requires safety to be setup and this is false, it should panic
 */
extern bool safety_is_setup;

/**
 * @brief Boolean for if safety_init has been called
 * If code requires safety to be initialized and this is false, it should panic
 */
extern bool safety_initialized;

/**
 * @brief Performs core safety setup to be completed immediately after reset
 * This enables a watchdog timer, but with a multi-second long tick for setup
 * Will also print data on the last reset cause and any crash data from that
 * 
 * NOT INTERRUPT SAFE
 */
void safety_setup(void);

/**
 * @brief Initializes safety for normal robot operation
 * This will tighten the timing for the watchdog timer
 * 
 * NOT INTERRUPT SAFE
 * REQUIRES SETUP
 */
void safety_init(void);

/**
 * @brief Ticks safety
 * This must be called within the period of the watchdog timer or a reset will occur
 * 
 * NOT INTERRUPT SAFE
 * REQUIRES INITIALIZATION
 */
void safety_tick(void);





#endif