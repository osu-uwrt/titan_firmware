#ifndef _ACTUATOR_H
#define _ACTUATOR_H

#include "actuator_i2c/interface.h"

/**
 * @brief If the actuator interface is initialized
 */
extern bool actuator_initialized;

/**
 * @brief The last status message from the actuator board. Only valid if actuator is connected.
 */
extern struct actuator_i2c_status actuator_last_status;

/**
 * @brief Returns if the coprocessor is connected to the actuator board
 * 
 * @return true A connection is established
 * @return false No connection established or not initialized
 */
bool actuator_is_connected(void);

/**
 * @brief Initializes actuator board communication.
 */
void actuator_init(void);

/**
 * @brief Actuator commands
 * 
 * NOT INTERRUPT SAFE - Can allocate memory
 */
void actuator_open_claw(void);
void actuator_close_claw(void);
void actuator_set_claw_timings(uint16_t open_time_ms, uint16_t close_time_ms);
void actuator_arm_torpedo(void);
void actuator_disarm_torpedo(void);
void actuator_fire_torpedo(uint8_t torpedo_id);
void actuator_set_torpedo_timings(uint8_t torpedo_id, enum torpedo_timing_type timing_type, uint16_t time_us);
void actuator_drop_marker(uint8_t marker_id);
void actuator_clear_dropper_status(void);
void actuator_set_marker_timings(uint16_t active_time_ms);
void actuator_reset_actuators(void);

#endif