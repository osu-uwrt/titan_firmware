#ifndef _ACTUATORS__TORPEDO_H
#define _ACTUATORS__TORPEDO_H

#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_TORPEDO, Enable/disable assertions in the Torpedo module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_TORPEDO
#define PARAM_ASSERTIONS_ENABLED_TORPEDO 0
#endif

enum torpedo_state {
    TORPEDO_STATE_UNINITIALIZED,
    TORPEDO_STATE_DISARMED,
    TORPEDO_STATE_CHARGING,
    TORPEDO_STATE_READY,
    TORPEDO_STATE_FIRING,
};

enum torpedo_timing_type {
    ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME = 0,
    ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME = 1,
    ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME = 2,
    ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME = 3,
    ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME = 4,

    ACTUATOR_NUM_TORPEDO_TIMINGS
};

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
bool torpedo_set_timings(uint8_t torpedo_num, enum torpedo_timing_type timing_type, uint16_t time_us);

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
 * @return bool whether or not the torpedo armed
 */
bool torpedo_arm(void);

/**
 * @brief Attempts to disarm the torpedos
 *
 * @return bool twhether or not the torpedo disarmed
 */
bool torpedo_disarm(void);

/**
 * @brief gets the armed_state of the torpedos
 *
 * @return enum the torpedos current armed_state
 */
enum armed_state torpedo_get_armed_state(void);

/**
 * @brief Attempts to fire the requested torpedo
 *
 * @param torpedo_id the id of the torpedo to fire
 * @return if the torpedo successfully fired
 */
bool torpedo_fire(uint8_t torpedo_num);

/**
 * @brief Callback to disable torpedos when kill switch is removed from safety
 */
void torpedo_safety_disable(void);

#endif