#include "ros.h"
#include "safety_interface.h"
#include "basic_logger/logging.h"
#include "pico/stdlib.h"
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include "safety/safety.h"

const char * const entity_lookup_table[] = {
    "RMW_UROS_ERROR_ON_UNKNOWN",
    "RMW_UROS_ERROR_ON_NODE",
    "RMW_UROS_ERROR_ON_SERVICE",
    "RMW_UROS_ERROR_ON_CLIENT",
    "RMW_UROS_ERROR_ON_SUBSCRIPTION",
    "RMW_UROS_ERROR_ON_PUBLISHER",
    "RMW_UROS_ERROR_ON_GRAPH",
    "RMW_UROS_ERROR_ON_GUARD_CONDITION",
    "RMW_UROS_ERROR_ON_TOPIC",
};
const char * const source_lookup_table[] = {
    "RMW_UROS_ERROR_ENTITY_CREATION",
    "RMW_UROS_ERROR_ENTITY_DESTRUCTION",
    "RMW_UROS_ERROR_CHECK",
    "RMW_UROS_ERROR_NOT_IMPLEMENTED",
    "RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION",
};

#define lookup_string_enum(value, list) ((value < sizeof(list)/sizeof(*list)) && (value >= 0) ? list[value] : "Out-of-Bounds")
#define lookup_entity_enum(value) lookup_string_enum(value, entity_lookup_table)
#define lookup_source_enum(value) lookup_string_enum(value, source_lookup_table)

#define EXECUTOR_HANDLES 10
#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"

bool ros_connected = false;

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t heartbeat_publisher;
rcl_subscription_t killswtich_subscriber;

std_msgs__msg__Int8 heartbeat_msg;
std_msgs__msg__Bool killswitch_msg;

int failed_heartbeats = 0;

void rmw_error_cb(
  const rmw_uros_error_entity_type_t entity,
  const rmw_uros_error_source_t source,
  const rmw_uros_error_context_t context,
  const char * file,
  const int line) {
    printf("RMW UROS Error:\n\tEntity: %s\n\tSource: %s\n\tDesc: %s\n\tLocation: %s:%d\n", lookup_entity_enum(entity), lookup_source_enum(source), context.description, file, line);
}

void ros_rmw_init(void)  {
    rmw_uros_set_error_handling_callback(rmw_error_cb);
}

static void killswitch_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    LOG_INFO("New killswitch status: %d", msg->data);
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

rcl_ret_t ros_heartbeat_pulse() {
    heartbeat_msg.data = CAN_BUS_CLIENT_ID;
    rcl_ret_t ret = rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL);
    if (ret != RCL_RET_OK) {
        failed_heartbeats++;

        if(failed_heartbeats > MAX_MISSSED_HEARTBEATS) {
            ros_connected = false;
        }
    } else {
        failed_heartbeats = 0;
    }

    RCRETCHECK(ret);

    return true;
}

bool ros_init(const char *node_name, const char *namespace) {
        allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCRETCHECK(rclc_node_init_default(&node, BOARD_NAMESPACE, ROBOT_NAMESPACE, &support));

    RCRETCHECK(rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator));

    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));

    // BOARD SPECIFIC CODE HERE
    return true;
}

void ros_update(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    // BOARD SPECIFIC CODE HERE

    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    ros_connected = false;
}

bool is_ros_connected(void) {
    return ros_connected;
}

bool ros_ping(void) {
    ros_connected = rmw_uros_ping_agent(RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT, 1) == RCL_RET_OK;
    return ros_connected;
}

