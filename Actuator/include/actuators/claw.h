#ifndef _ACTUATORS__CLAW_H
#define _ACTUATORS__CLAW_H

#include <stdbool.h>
#include "actuator_i2c/interface.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_CLAW, Enable/disable assertions in the claw module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_CLAW
#define PARAM_ASSERTIONS_ENABLED_CLAW 0
#endif

/**
 * @brief Boolean for if the claw hardware has been initialized
 */
extern bool claw_initialized;

/**
 * @brief Initializes the claw hardware
 */
void claw_initialize(void);

/**
 * @brief Attempts to set the claw timings
 * 
 * @param timings Message struct containing claw timings
 * @return enum actuator_command_result Result on setting the claw timings
 */
enum actuator_command_result claw_set_timings(struct claw_timing_cmd *timings);

/**
 * @brief Populates the passed missing timing struct with the claw data
 * 
 * @param missing_timings The struct to populate
 */
void claw_populate_missing_timings(struct missing_timings_status* missing_timings);

/**
 * @brief Returns the current state of the claw
 * 
 * @return enum claw_state Claw's Current State
 */
enum claw_state claw_get_state(void);

/**
 * @brief Attempts to begin opening the claw
 * 
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result claw_open(void);

/**
 * @brief Attempts to begin closing the claw
 * 
 * @return enum actuator_command_result Result for the attempted command
 */
enum actuator_command_result claw_close(void);

/**
 * @brief Callback to disable claw when kill switch is removed from safety
 */
void claw_safety_disable(void);

#endif