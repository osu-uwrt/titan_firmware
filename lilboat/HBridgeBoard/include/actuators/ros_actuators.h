#ifndef ROS_ACTUATORS_H
#define ROS_ACTUATORS_H

#include "actuators/hiwonder_driver.h"
#include "ros.h"

extern servo_t claw_grip;

extern const size_t ros_actuators_num_executor_handles;

extern void servo_ping_all();

extern rcl_ret_t ros_actuators_update_status();

extern rcl_ret_t ros_actuators_update_cmd_feedback();

extern rcl_ret_t ros_update_actuator_degrees();

extern rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node);

extern rcl_ret_t ros_actuators_fini(rcl_node_t *node);

extern void init_servos();

#endif  // ROS_ACTUATORS_H
