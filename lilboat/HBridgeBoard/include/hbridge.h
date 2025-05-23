#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "pico/stdlib.h"

void hbridge_set_target(uint idx, float target_pct);

void hbridge_set_enabled(bool enabled);

// Returns the number of bridges woken
uint hbridge_wake();

void hbridge_sleep();

// Returns the bridge index
uint hbridge_create(uint ph_pin, uint en_pin, uint nfault_access, bool multiplex_nfault);

void hbridge_init(uint num_bridges, uint nsleep_pin, uint drvoff_pin);

#endif
