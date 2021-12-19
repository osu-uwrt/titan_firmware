#ifndef _SAFETY_H
#define _SAFETY_H

#include <stdbool.h>

/**
 * @brief Boolean for if safety_init has been called
 * If code requires safety to be initialized and this is false, it should panic
 */
bool safety_initialized;

/**
 * @brief Performs core safety setup to be completed immediately after reset
 * This enables a watchdog timer, but with a multi-second long tick for setup
 * Will also print data on the last reset cause and any crash data from that
 */
void safety_setup(void);

/**
 * @brief Initializes safety for normal robot operation
 * This will tighten the timing for the watchdog timer
 */
void safety_init(void);

/**
 * @brief Ticks safety
 * This must be called within the period of the watchdog timer or a reset will occur
 */
void safety_tick(void);

#endif