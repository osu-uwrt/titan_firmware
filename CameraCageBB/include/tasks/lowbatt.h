#ifndef _LOWBATT_H
#define _LOWBATT_H

/**
 * @brief The minimum disable time for when low battery state is entered
 */
#define LOWBATT_MIN_DISABLE_TIME 5000

/**
 * @brief The cutoff voltage for low battery
 */
#define LOWBATT_CUTOFF_VOLTAGE 18.5

/**
 * @brief The voltage for if a battery should be considered present
 */
#define LOWBATT_PRESENT_VOLTAGE 5.0

/**
 * @brief If the low battery task has been initialized
 */
extern bool lowbatt_initialized;

/**
 * @brief Ticks the low battery task
 * 
 * INITIALIZATION REQUIRED
 */
void lowbatt_tick(void);

/**
 * @brief Initializes low battery task
 */
void lowbatt_init(void);

#endif