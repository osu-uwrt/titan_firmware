#ifndef _ESC_PWM_H
#define _ESC_PWM_H

#if HW_USE_PWM

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ESC_PWM, Enable/disable assertions in the esc pwm module, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_ESC_PWM
#define PARAM_ASSERTIONS_ENABLED_ESC_PWM 0
#endif

#define ESC_PWM_COMMAND_MAX_TIME_DIFF_MS 250
#define ESC_PWM_MIN_UPDATE_RATE_MS 1000
#define ESC_PWM_UPDATE_DISABLE_TIME_MS 1000
#define ESC_PWM_WAKEUP_DELAY_MS 5000

#include <stdint.h>
#include <riptide_msgs2/msg/pwm_stamped.h>

/**
 * @brief Boolean if esc_pwm_init has been called
 */
extern bool esc_pwm_initialized;

/**
 * @brief Sends stop command to all thrusters.
 * If esc pwm has not been initialized yet, this call does nothing
 */
void esc_pwm_stop_thrusters(void);

/**
 * @brief Updates thruster values to the specified commands.
 * Note if this is not updated enough the thrusters will time out
 * 
 * @param thruster_commands 
 */
void esc_pwm_update_thrusters(const riptide_msgs2__msg__PwmStamped *thruster_commands);

/**
 * @brief Sets the robot into low battery state which will disable thrusters
 * 
 * @param in_lowbatt_state 
 */
void esc_pwm_set_lowbatt(bool in_lowbatt_state);

/**
 * @brief Initialize pwm and starts outputting thruster neutral commands
 */
void esc_pwm_init(void);

#endif  // HW_USE_PWM

#endif  // _ESC_PWM_H