#ifndef ROS__ROS_INTERNAL_H
#define ROS__ROS_INTERNAL_H

#include "ros.h"

#include "titan/logger.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Topic Names
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define LED_SUBSCRIBER_NAME "command/led"
#define PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME "state/physkill_notify"
#define ELECTRICAL_COMMAND_SUBSCRIBER_NAME "command/electrical"

#define BUSY_TOPIC_NAME "state/actuator/busy"
#define STATUS_TOPIC_NAME "state/actuator/status"
#define DYNAMIXEL_STATUS_TOPIC_NAME "state/actuator/dynamixel"
#define TORPEDO_SERVICE_NAME "command/actuator/torpedo"
#define DROPPER_SERVICE_NAME "command/actuator/dropper"
#define CLAW_SERVICE_NAME "command/actuator/claw"
#define NOTIFY_RELOAD_SERVICE_NAME "command/actuator/notify_reload"
#define ARM_SERVICE_NAME "command/actuator/arm"
#define TORPEDO_MARKER_MOVE_HOME_SERVICE_NAME "command/actuator/torpedo_marker/go_home"
#define TORPEDO_MARKER_SET_HOME_SERVICE_NAME "command/actuator/torpedo_marker/set_home"

// ========================================
// Internal Functions
// ========================================

extern const size_t ros_actuators_num_executor_handles;
rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node);
void ros_actuators_fini(rcl_node_t *node);

#if ACTUATOR_V1_SUPPORT
extern const size_t actuator_v1_parameters_num_executor_handles;
rcl_ret_t actuator_v1_parameters_init(rcl_node_t *node, rclc_executor_t *executor);
void actuator_v1_parameters_fini(rcl_node_t *node);
#endif

#if ACTUATOR_V2_SUPPORT
extern const size_t actuator_v2_dynamixel_num_executor_handles;
rcl_ret_t actuator_v2_dynamixel_update_status(void);
rcl_ret_t actuator_v2_dynamixel_init(rcl_node_t *node, rclc_executor_t *executor);
void actuator_v2_dynamixel_fini(rcl_node_t *node);
#endif

#endif
