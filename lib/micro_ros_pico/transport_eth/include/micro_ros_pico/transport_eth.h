#ifndef MICRO_ROS_PICO__TRANSPORT_ETH_H
#define MICRO_ROS_PICO__TRANSPORT_ETH_H

#include <stdint.h>

void transport_eth_init(int sock_num, uint32_t target_ip, uint16_t target_port);

#endif //MICRO_ROS_PICO__TRANSPORT_ETH_H
