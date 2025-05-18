#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "pico/stdlib.h"

void hbridge_set_power(float power);

void hbridge_init(uint ph_pin, uint en_pin);

#endif
