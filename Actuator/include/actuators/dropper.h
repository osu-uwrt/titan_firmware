#ifndef _ACTUATORS__DROPPER_H
#define _ACTUATORS__DROPPER_H

#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DROPPER, Enable/disable assertions in the Dropper module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_DROPPER
#define PARAM_ASSERTIONS_ENABLED_DROPPER 0
#endif

enum dropper_state {
    DROPPER_STATE_UNINITIALIZED,
    DROPPER_STATE_READY,
    DROPPER_STATE_DROPPING,
};

/**
 * @brief Boolean for if the marker dropper hardware has been initialized
 */
extern bool dropper_initialized;

/**
 * @brief Initializes the marker dropper hardware
 */
void dropper_initialize(void);

/**
 * @brief Attempts to set the marker dropper timings
 *
 * @param uint16_t active_time_ms dropper timings
 * @return bool Result of setting the dropper timings
 */
bool dropper_set_timings(uint16_t active_time_ms);

/**
 * @brief Populates the passed missing timing struct with the marker dropper data
 *
 * @param missing_timings The struct to populate
 */
void dropper_populate_missing_timings(struct missing_timings_status* missing_timings);

/**
 * @brief Returns the current state of the marker dropper
 *
 * @param dropper_id: The dropper to get the state of (id starts at 1)
 * @return enum dropper_state dropper's Current State
 */
enum dropper_state dropper_get_state(uint8_t dropper_id);

/**
 * @brief Attempts to drop the requested marker
 *
 * @param dropper_id: The dropper to drop (id starts at 1)
 * @return bool Result for the attempted command
 */
bool dropper_drop_marker(uint8_t dropper_id);

/**
 * @brief Attempts to clear the status for the marker dropper back to not dropped
 *
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result dropper_clear_status(void);

/**
 * @brief Callback to disable dropper when kill switch is removed from safety
 */
void dropper_safety_disable(void);

/**
 * @brief arms the dropper
 *
 * @return bool if the dropper was armed (false if already armed)
 */
bool dropper_arm(void);

/**
 * @brief disarms the dropper
 *
 * @return bool if the dropper was disarmed (false if already disarmed)
 */
bool dropper_disarm(void);

/**
 * @brief gets the armed_state of the dropper
 *
 * @return enum the droppers current armed_state
 */
enum armed_state dropper_get_armed_state(void);

#endif