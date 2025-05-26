#ifndef CD74HC4051_H
#define CD74HC4051_H

#include "pico/stdlib.h"

/**
 * @brief Returns the selected ADC reading.
 *
 * @param num The [0, 7] number multiplexer input to select
 *
 * @return float The selected ADC reading in volts, on [0, 3.3]
 */
float multiplexer_decode_analog(uint num);

/**
 * @brief Returns a boolean representation of the selected pin's reading.
 *
 * @param num The [0, 7] number multiplexer input to select
 *
 * @return bool True if the pin is at or above LOGIC_HIGH, false otherwise
 */
bool multiplexer_decode_digital(uint num);

/**
 * @brief Initializes a TI CD74HC4051 8-channel multiplexer.
 *
 * @param read_pin Data read pin. Must be an ADC pin
 * @param s0_pin LSB selection pin
 * @param s1_pin Middle selection pin
 * @param s2_pin MSB selection pin
 *
 * @attention This function should only be called once
 */
void multiplexer_init(uint read_pin, uint s0_pin, uint s1_pin, uint s2_pin);

#endif  // CD74HC4051_H
