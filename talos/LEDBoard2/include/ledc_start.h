#ifndef LED_START_H
#define LED_START_H

#define LEDC_SPI_INST __CONCAT(spi, LEDC_SPI)

/**
 * @brief initialize the spi hardware and pins so we can communicate with the led controllers
 *
 */
void init_spi(void);

/**
 * @brief initialize the PWM_CLK used by the led controllers
 *
 */
void init_pwm(void);

/**
 * @brief initialize all the hardware (gpio pins, pwm and spi on respective pins)
 *
 */
void init_hardware(void);
#endif  // LED_START_H
