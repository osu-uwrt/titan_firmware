#ifndef MICRO_ROS_PICO__TRANSPORT_ETH_H
#define MICRO_ROS_PICO__TRANSPORT_ETH_H

#include <stdint.h>

// PICO_CONFIG: MICRO_ROS_PICO_ETH_PORT, UDP port for micro ros communication, type=int, min=1, max=65534, default=24321, group=micro_ros_pico
#ifndef MICRO_ROS_PICO_ETH_PORT
#define MICRO_ROS_PICO_ETH_PORT 24321
#endif

// PICO_CONFIG: MICRO_ROS_PICO_ETH_SOCK_NUM, Socket number to use on the wiznet chip for micro ros communication, type=int, min=0, max=3, default=0, group=micro_ros_pico
#ifndef MICRO_ROS_PICO_ETH_SOCK_NUM
#define MICRO_ROS_PICO_ETH_SOCK_NUM 0
#endif

void transport_eth_init(void);

#endif //MICRO_ROS_PICO__TRANSPORT_ETH_H
