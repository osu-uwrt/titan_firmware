#ifndef MICRO_ROS_PICO__TRANSPORT_ETH_H
#define MICRO_ROS_PICO__TRANSPORT_ETH_H

#include <stdint.h>

#include "eth_networking.h"

// PICO_CONFIG: MICRO_ROS_PICO_ETH_SOCK_NUM, Socket number to use on the wiznet chip for micro ros communication, type=int, min=0, max=3, default=0, group=micro_ros_pico
#ifndef MICRO_ROS_PICO_ETH_SOCK_NUM
#define MICRO_ROS_PICO_ETH_SOCK_NUM 0
#endif

bool transport_eth_init(w5k_data_t * eth_device);

bool ethernet_check_online(w5k_data_t * eth_device);

#endif //MICRO_ROS_PICO__TRANSPORT_ETH_H
