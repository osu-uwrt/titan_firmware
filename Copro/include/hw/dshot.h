#ifndef _DSHOT_H
#define _DSHOT_H

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
 * @brief Initialize dshot and starts PIO engine outputting stop thruster commands
 */
void dshot_init(void);

#endif