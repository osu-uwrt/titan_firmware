#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "pico/stdlib.h"

#define SERVO_MIN_DEG 0
#define SERVO_MAX_DEG 240

extern void servo_ping();

extern void servo_set_armed(bool armed);

extern uint16_t servo_set_deg(float deg);

extern void servo_set_deg_then_home(float deg);

extern void servo_read_deg();

extern void servo_go_home();

extern void servo_set_home();

extern void init_servo();

extern volatile bool enabled;
extern volatile bool move_active;
extern volatile bool connected;
extern volatile bool homed;

#endif  // ACTUATOR_H
