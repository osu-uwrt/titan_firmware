#ifndef ROBOTS__TEMPEST_H_
#define ROBOTS__TEMPEST_H_

#ifdef UWRT_ROBOT_DEFINED
#error Multiple robot types defined
#endif
#define UWRT_ROBOT_DEFINED

// ROS Namespace
#define ROBOT_NAMESPACE "tempest"

// Address Definitions
#define ROBOT_COMPUTER_IP                                                                                              \
    { 192, 168, 1, 23 }
#define ROBOT_COMPUTER_UROS_PORT 8888

#define ETHERNET_MASK                                                                                                  \
    { 255, 255, 255, 0 }
#define ETHERNET_GATEWAY                                                                                               \
    { 192, 168, 1, 1 }
#define ETHERNET_BUS_ID 0

#endif
