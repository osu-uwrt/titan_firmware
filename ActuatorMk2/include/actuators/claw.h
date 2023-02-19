#ifndef _ACTUATORS__CLAW_H
#define _ACTUATORS__CLAW_H

#include <stdbool.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_CLAW, Enable/disable assertions in the claw module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_CLAW
#define PARAM_ASSERTIONS_ENABLED_CLAW 0
#endif

enum claw_state {
    CLAW_STATE_UNINITIALIZED = 0,
    CLAW_STATE_UNKNOWN_POSITION = 1,
    CLAW_STATE_OPENED = 2,
    CLAW_STATE_CLOSED = 3,
    CLAW_STATE_OPENING = 4,
    CLAW_STATE_CLOSING = 5,
    CLAW_STATE_UNPLUGGED = 6,
    CLAW_STATE_MOTOR_DRIVER_FAULT = 7,
};

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
bool claw_set_timings(uint16_t open_time_ms, uint16_t close_time_ms);

/**
 * @brief Returns the current state of the claw
 *
 * @return enum claw_state Claw's Current State
 */
enum claw_state claw_get_state(void);

/**
 * @brief Attempts to begin opening the claw
 *
 * @return bool true when opened or running, false otherwise.
 */
bool claw_open(void);

/**
 * @brief Attempts to begin closing the claw
 *
 * @return bool true when opened or running, false otherwise.
 */
bool claw_close(void);

/**
 * @brief Callback to disable claw when kill switch is removed from safety
 */
void claw_safety_disable(void);

#endif