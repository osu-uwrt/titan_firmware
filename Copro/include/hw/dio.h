#ifndef _DIO_H
#define _DIO_H

/**
 * @brief Boolean if dio_init has been called
 */
bool dio_initialized;

/**
 * @brief Sets the fault led. If dio has not been initialized yet, it will store the state for when dio is initialized
 * 
 * INTERRUPT SAFE
 * 
 * @param on The target led state
 */
void dio_set_fault_led(bool on);

/**
 * @brief Initializes digital io required by the firmware
 * The digital io deals with any raw digital signals that need to be procesed by the firmware
 * 
 * NOT INTERRUPT SAFE
 */
void dio_init(void);

#endif