#ifndef ROS__ROS_INTERNAL_H
#define ROS__ROS_INTERNAL_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

extern const size_t ros_actuators_num_executor_handles;
rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node, __unused rclc_support_t *support);
void ros_actuators_fini(rcl_node_t *node);

#endif