#pragma once

/**
 * @brief Initializes the depth sensors (both i2c and ADCs)
 */
void depth_init();

/**
 * @brief Reads the depth from the ADC
 *
 * @param which depth ADC to read (either 0 or 1)
 */
float depth_adc_read(int adc);
