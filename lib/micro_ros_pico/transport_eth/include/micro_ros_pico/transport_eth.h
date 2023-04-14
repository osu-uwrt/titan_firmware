#ifndef MICRO_ROS_PICO__TRANSPORT_ETH_H
#define MICRO_ROS_PICO__TRANSPORT_ETH_H

// PICO_CONFIG: ETH_HEARTBEAT_INTERVAL_MS, Interval for CANmore heartbeat transmission over CAN bus in milliseconds, type=int, default=500, group=transport_eth
#ifndef ETH_HEARTBEAT_INTERVAL_MS
#define ETH_HEARTBEAT_INTERVAL_MS 500
#endif

#include <stdint.h>

bool transport_eth_init();

bool ethernet_check_online();
void ethernet_tick();

#endif //MICRO_ROS_PICO__TRANSPORT_ETH_H
