#ifndef ROS_TORP_H
#define ROS_TORP_H

#include "ros.h"

extern const size_t ros_actuators_num_executor_handles;

extern rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node);

extern rcl_ret_t ros_actuators_fini(rcl_node_t *node);

extern rcl_ret_t ros_actuators_update_status(void);

extern rcl_ret_t ros_actuators_update_cmd_feedback(void);

extern void torpedo_init_servo(void);

extern void torpedo_ping_servo(void);

#endif  // ROS_TORP_H
