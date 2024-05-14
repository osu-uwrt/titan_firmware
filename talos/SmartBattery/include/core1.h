#ifndef CORE1_H
#define CORE1_H

#include "bq40z80.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum batt_state {
    BATT_STATE_UNINITIALIZED,   // core1_init has not been called yet
    BATT_STATE_DISCONNECTED,    // No connection to the battery
    BATT_STATE_REMOVED,         // The pack is not installed in anything right now
    BATT_STATE_NEEDS_SHUTDOWN,  // The bq40 is trying to shut itself off, the MCU should stop waking it and do the same
    BATT_STATE_INITIALIZING,    // The bq40 is initializing after startup
    BATT_STATE_XDSG,            // The bq40 is plugged in, but discharging is disabled for whatever reason
    BATT_STATE_DISCHARGING,     // The pack is currently discharging
    BATT_STATE_CHARGING,        // The pack is currently charging
    BATT_STATE_PERMENANT_FAIL,  // The pack is in a permenant failure state, it is inoperable and requires disassembly
    BATT_STATE_POWER_CYCLE,     // The pack is currently power cycling
    BATT_STATE_LATCH_OFF        // The pack is in a latched off state, waiting for wakeup or unplug
} batt_state_t;

void core1_init(uint8_t expected_serial);
bool core1_get_pack_mfg_info(bq_mfg_info_t *pack_info_out);
batt_state_t core1_get_batt_state(void);

// ========================================
// Battery Info Query
// ========================================

// These are pulled from the last poll to the device. Only valid if batt state isn't in UNITIALIZED or DISCONNECTED
int32_t core1_avg_current(void);
uint8_t core1_soc(void);
uint16_t core1_time_remaining(bool *is_charging);
uint16_t core1_voltage(void);
int32_t core1_current(void);
void core1_cell_voltages(uint16_t *cell_voltages_out);

// ========================================
// Commands
// ========================================

/**
 * @brief Sends request to query the side detect state.
 * Repeatedly call this function until it returns true.
 *
 * @details
 * This function works by sending a request to read the register if no requests are in progress. Once the result is
 * successfully read by the other core, it is unread, and this function will return true. The next call to this function
 * after it returns true will queue a new request.
 *
 * To avoid reading stale data, if the first call to this function returns true, call the function again to ensure that
 * the data is fresh.
 *
 * @param side_det_is_high Pointer to write the side detect pin value upon returning true
 * @return true The request is finished and the result has been written to the output pointer
 * @return false The request is still in progress. Call this function again
 */
bool core1_get_side_detect(bool *side_det_is_high);

/**
 * @brief Sends command to power cycle the robot
 */
void core1_power_cycle_robot(void);

/**
 * @brief Sends command to latch off robot power until restore command is sent
 */
void core1_kill_robot_power(void);

/**
 * @brief Restores robot power after a kill_robot_power call
 */
void core1_restore_robot_power(void);

#endif
