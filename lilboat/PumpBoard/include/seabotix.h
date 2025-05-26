#ifndef SEABOTIX_H
#define SEABOTIX_H

#include "pico/stdlib.h"

#define SEABOTIX_PWM_HZ 50000

void seabotix_init();
void seabotix_disable();
void seabotix_set_enable_state(bool state);
void seabotix_set_pct(uint8_t target, int8_t pct);
void seabotix_set_pct_for(uint8_t target, int8_t pct, uint time_ms);

#endif  // SEABOTIX_H
