#ifndef ROS__ROS_INTERNAL_H
#define ROS__ROS_INTERNAL_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "ros.h"
#include "basic_logger/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

extern const size_t ros_actuators_num_executor_handles;
rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node, __unused rclc_support_t *support);
void ros_actuators_fini(rcl_node_t *node);

#if ACTUATOR_V1_SUPPORT
extern const size_t actuator_v1_parameters_num_executor_handles;
rcl_ret_t actuator_v1_parameters_init(rcl_node_t *node, rclc_executor_t *executor);
void actuator_v1_parameters_fini(rcl_node_t *node);
#endif

#endif