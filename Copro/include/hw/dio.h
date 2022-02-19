#ifndef _DIO_H
#define _DIO_H

/**
 * @brief Time to toggle power rails for
 * This is used for cases where toggling the power rail would disconnect the computer
 * preventing the computer from being able to instruct it to be turned back on.
 */
#define POWER_RAIL_TOGGLE_TIME_MS 500

/**
 * @brief Boolean if dio_init has been called
 */
extern bool dio_initialized;

/**
 * @brief Sets the fault led. If dio has not been initialized yet, it will store the state for when dio is initialized
 * 
 * INTERRUPT SAFE
 * 
 * @param on The target led state
 */
void dio_set_fault_led(bool on);

/**
 * @brief Returns the state of the aux switch
 * 
 * @return true Aux switch is placed on robot
 * @return false Aux switch is removed from robot
 */
bool dio_get_aux_switch(void);

#ifdef REG_12_CTRL_PIN
/**
 * @brief Lowers the 12V power rail for POWER_RAIL_TOGGLE_TIME_MS and then raises is again
 */
void dio_toggle_twelve_power(void);
#endif

/**
 * @brief Lowers the 5V power rail for POWER_RAIL_TOGGLE_TIME_MS and then raises is again
 */
void dio_toggle_five_power(void);

/**
 * @brief Lowers the Mobo power for POWER_RAIL_TOGGLE_TIME_MS and then raises is again
 */
void dio_toggle_mobo_power(void);

/**
 * @brief Sets the peltier control power line to the passed in value
 * 
 * @param on The value to set the peltier power to
 */
void dio_set_peltier_power(bool on);

/**
 * @brief Initializes digital io required by the firmware
 * The digital io deals with any raw digital signals that need to be procesed by the firmware
 * 
 * NOT INTERRUPT SAFE
 */
void dio_init(void);

#endif