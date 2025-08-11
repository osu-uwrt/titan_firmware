#pragma once

#include <stdbool.h>

#define SOLENOID_COUNT 4

/**
 * @brief Initializes the solenoid pins
 */
void solenoid_init();

/**
 * @brief Sets a solenoid to open or closed
 *
 * @param number the number of solenoid (in the range [1..SOLENOID_COUNT])
 */
void solenoid_set(int number, bool open);

bool solenoid_get(int number);
