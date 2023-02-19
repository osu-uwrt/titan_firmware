#ifndef LED_H
#define LED_H

#include <rcl/rcl.h>

void led_init();

void led_fault_set(bool value);

void led_can_set(bool value);

void led_enabled_set(bool value);

void led_killswitch_set(bool value);

rcl_ret_t led_update_can();

#endif