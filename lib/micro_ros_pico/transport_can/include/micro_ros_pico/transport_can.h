#ifndef MICRO_ROS_PICO__TRANSPORT_CAN_H
#define MICRO_ROS_PICO__TRANSPORT_CAN_H

#include <stdint.h>

#include "hardware/spi.h"

void transport_can_init(uint client_id);

#endif //MICRO_ROS_PICO__TRANSPORT_CAN_H
