#ifndef _COOLING_H
#define _COOLING_H

/**
 * @brief The maximum age of the adc reading before treating as invalid
 */
#define COOLING_MAX_READING_AGE_MS 2000

/**
 * @brief If the cooling task has been initialized
 */
extern bool cooling_initialized;

/**
 * @brief Threshold when peltier panel turns on in Deg C
 */
extern int cooling_threshold;

/**
 * @brief Ticks the cooling task
 * 
 * INITIALIZATION REQUIRED
 */
void cooling_tick(void);

/**
 * @brief Returns if the robot is being actively cooled
 * 
 * @return true Peltier panel is on
 * @return false Peltier panel is off
 */
bool cooling_get_active(void);

/**
 * @brief Initializes cooling task
 */
void cooling_init(void);

#endif