#ifndef LEDC_COMMANDS_H_
#define LEDC_COMMANDS_H_

#define LEDC1 0
#define LEDC2 1

#define BUCK1 0
#define BUCK2 1

#define BUCK_ALWAYS_OFF 0
#define BUCK_PWM_DIMMING 1
#define BUCK_ALWAYS_ON 2
#define BUCK_FOLLOW_DIN 3

/**
 * @brief Possible operation modes for the LEDs.
 */
enum status_mode { MODE_SOLID, MODE_SLOW_FLASH, MODE_FAST_FLASH, MODE_BREATH };

_Bool controller_satisfy_watchdog();
void controller_clear_watchdog_error(unsigned int controller);
void controller_enable(unsigned int controller);

void buck_set_control_mode(unsigned int controller, unsigned int buck, unsigned int mode);

void init_spi_and_gpio();

#endif
