#ifndef _DSHOT_H
#define _DSHOT_H

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DSHOT, Enable/disable assertions in the DSHOT module, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_DSHOT
#define PARAM_ASSERTIONS_ENABLED_DSHOT 0
#endif

#define DSHOT_MAX_UPDATE_RATE_MS 50
#define DSHOT_UPDATE_DISABLE_TIME_MS 10000

/**
 * @brief Boolean if dshot_init has been called
 */
bool dshot_initialized;

/**
 * @brief Sends stop command to all thrusters.
 * If dshot has not been initialized yet, this call does nothing
 */
void dshot_stop_thrusters(void);

/**
 * @brief Notifies dshot controller logic of a change on the physical kill switch line.
 * This is needed to re-initialize the ESCs in the event of the power being lost
 * from the kill switch.
 * 
 * @param new_value The new value of the kill switch line
 */
void dshot_notify_physical_kill_switch_change(bool new_value);

/**
 * @brief Updates thruster values to the specified commands.
 * Note if this is not updated enough the thrusters will time out
 * 
 * @param thruster_commands 
 */
void dshot_update_thrusters(uint16_t *thruster_commands);

/**
 * @brief Sets the robot into low battery state which will disable thrusters
 * 
 * @param in_lowbatt_state 
 */
void dshot_set_lowbatt(bool in_lowbatt_state);

/**
 * @brief Initialize dshot and starts PIO engine outputting stop thruster commands
 */
void dshot_init(void);

#endif