#pragma once

/**
 * @brief Initializes the pressure sensors (both i2c and ADCs)
 */
void pressure_init();

/**
 * @brief Reads the pressure from the ADC
 *
 * @param adc which ADC to read (either 0 or 1)
 */
float pressure_read_adc(int adc);
