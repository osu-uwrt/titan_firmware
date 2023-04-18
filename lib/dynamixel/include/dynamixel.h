#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdlib.h>

typedef uint8_t dynamixel_id;

typedef void (*dynamixel_error_cb)(uint8_t error_code);

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb);

void dynamixel_set_id();

void dynamixel_ping();

void dynamixel_reboot();

void dynamixel_factory_reset();

/* How control servo (?) */

#endif