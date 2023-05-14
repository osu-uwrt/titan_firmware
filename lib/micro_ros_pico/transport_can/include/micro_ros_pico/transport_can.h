#ifndef MICRO_ROS_PICO__TRANSPORT_CAN_H_
#define MICRO_ROS_PICO__TRANSPORT_CAN_H_

#include <stdint.h>
#include "micro_ros_pico/common.h"

/**
 * @brief Initialize Micro-ROS using the CAN Transport
 * This handles the initialization of the CAN bus hardware
 *
 * @param client_id The client id to use for CAN traffic
 * @return true CAN hardware was successfully initialized and the transport was set
 * @return false CAN hardware failed to initialize
 */
bool transport_can_init(uint8_t client_id);

#endif //MICRO_ROS_PICO__TRANSPORT_CAN_H
