#ifndef _ACTUATORS__TORPEDO_H
#define _ACTUATORS__TORPEDO_H

#include <stdint.h>
#include "actuator_i2c/interface.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_TORPEDO, Enable/disable assertions in the Torpedo module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_TORPEDO
#define PARAM_ASSERTIONS_ENABLED_TORPEDO 0
#endif

/**
 * @brief Boolean for if the torpedo hardware has been initialized
 */
extern bool torpedo_initialized;

/**
 * @brief Initializes the torpedo hardware
 */
void torpedo_initialize(void);

/**
 * @brief Attempts to set the torpedo timings
 * 
 * @param timings Message struct containing torpedo timings
 * @return enum actuator_command_result Result on setting the torpedo timings
 */
enum actuator_command_result torpedo_set_timings(struct torpedo_timing_cmd *timings);

/**
 * @brief Populates the passed missing timing struct with the torpedo data
 * 
 * @param missing_timings The struct to populate
 */
void torpedo_populate_missing_timings(struct missing_timings_status* missing_timings);

/**
 * @brief Returns the current state of the torpedo
 * 
 * @param torpedo_id: The torpedo to get the state of (id starts at 1)
 * @return enum torpedo_state torpedo's Current State
 */
enum torpedo_state torpedo_get_state(uint8_t torpedo_id);

/**
 * @brief Attempts to arm the torpedos
 * 
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result torpedo_arm(void);

/**
 * @brief Attempts to disarm the torpedos
 * 
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result torpedo_disarm(void);

/**
 * @brief Attempts to fire the requested torpedo
 * 
 * @param cmd The parameter for the torpedo_fire command (id starts at 1)
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result torpedo_fire(struct fire_torpedo_cmd *cmd);

/**
 * @brief Callback to disable torpedos when kill switch is removed from safety
 */
void torpedo_safety_disable(void);

#endif