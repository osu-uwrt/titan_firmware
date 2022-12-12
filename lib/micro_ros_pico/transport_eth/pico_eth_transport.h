#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

void pico_eth_transport_init(int sock_num, uint32_t target_ip, uint16_t target_port);

#endif //MICRO_ROS_PICOSDK
