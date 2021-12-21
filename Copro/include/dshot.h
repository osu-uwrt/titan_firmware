#ifndef _DSHOT_H
#define _DSHOT_H

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
 * @brief Initialize dshot and starts PIO engine outputting stop thruster commands
 */
void dshot_init(void);

#endif