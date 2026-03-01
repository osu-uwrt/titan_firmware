#ifndef ROS_CLAW_H
#define ROS_CLAW_H

#include "ros.h"

extern const size_t ros_claw_num_executor_handles;

extern rcl_ret_t ros_claw_init(rclc_executor_t *executor, rcl_node_t *node);

extern rcl_ret_t ros_claw_fini(rcl_node_t *node);

extern void claw_init_servo();

extern void claw_ping_servo();

#endif  // ROS_CLAW_H
