#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "hiwonder_driver.h"

#include "pico/stdlib.h"

#define SERVO_MIN_DEG 0
#define SERVO_MAX_DEG 240

extern void servo_ping();

extern void servo_set_armed(servo_t *servo, bool armed);

extern uint16_t servo_set_deg(servo_t *servo, float deg);

extern void servo_set_deg_then_home(servo_t *servo, float deg);

extern void servo_read_deg(servo_t *servo);

extern void servo_set_id(uint8_t old_id, uint8_t new_id);

extern void servo_read_id();

extern void servo_go_home(servo_t *servo);

extern void servo_set_home(servo_t *servo);

extern void init_servos();

extern void make_servo(servo_t *servo, uint16_t home_deg);

#endif  // ACTUATOR_H
