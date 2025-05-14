#pragma once

#include <stdbool.h>

#define SOLENOID_COUNT 3

/**
 *
 */
void solenoid_init();

void solenoid_set(int number, bool open);
