#ifndef _ACTUATORS__DROPPER_H
#define _ACTUATORS__DROPPER_H

#include <stdint.h>
#include "actuator_i2c/interface.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DROPPER, Enable/disable assertions in the Dropper module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_DROPPER
#define PARAM_ASSERTIONS_ENABLED_DROPPER 0
#endif

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
 * @param timings Message struct containing dropper timings
 * @return enum actuator_command_result Result on setting the dropper timings
 */
enum actuator_command_result dropper_set_timings(struct dropper_timing_cmd *timings);

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


#endif