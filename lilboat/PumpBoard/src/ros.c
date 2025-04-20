#include "ros.h"

#include "seabotix.h"

#include "driver/depth.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/depth.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include <std_srvs/srv/trigger.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define DEPTH_PUBLISHER_NAME "state/depth/raw"
#define LEFT_POWER_SUBSCRIBER_NAME "command/left"
#define RIGHT_POWER_SUBSCRIBER_NAME "command/right"
#define PRESSURE_PUBLISHER_NAME "pressure"
#define FLUSH_SERVICE_NAME "command/flush"

#define FLUSH_TARGET 0
#define FLUSH_TIME_MS 75

bool ros_connected = false;

// Core Variables
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t heartbeat_publisher;
int failed_heartbeats = 0;

// Node specific Variables
rcl_publisher_t firmware_status_publisher;
rcl_subscription_t killswtich_subscriber;
std_msgs__msg__Bool killswitch_msg;
// TODO: Add node specific items here
rcl_subscription_t left_power_subscriber;
std_msgs__msg__Int8 left_power_msg;
rcl_subscription_t right_power_subscriber;
std_msgs__msg__Int8 right_power_msg;

// Depth Sensor
rcl_publisher_t depth_publisher;
rcl_publisher_t water_temp_publisher;
riptide_msgs2__msg__Depth depth_msg;
char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
const float depth_variance = 0.003;

// Pressure
rcl_publisher_t pressure_publisher;

// Flush
rcl_service_t flush_service;
std_srvs__srv__Trigger_Request flush_service_req;
std_srvs__srv__Trigger_Response flush_service_res;

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void left_power_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *) msgin;
    seabotix_set_pct(0, msg->data);
}

static void right_power_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *) msgin;
    seabotix_set_pct(1, msg->data);
}

static void flush_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    seabotix_set_pct_for(FLUSH_TARGET, 100, FLUSH_TIME_MS);

    const char *message = "Triggering flush operation";
    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;

    res_in->success = true;
}

// TODO: Add in node specific tasks here

// ========================================
// Public Task Methods (called in main tick)
// ========================================

rcl_ret_t ros_update_firmware_status(uint8_t client_id) {
    riptide_msgs2__msg__FirmwareStatus status_msg;
    status_msg.board_name.data = PICO_BOARD;
    status_msg.board_name.size = strlen(PICO_BOARD);
    status_msg.board_name.capacity = status_msg.board_name.size + 1;  // includes NULL byte

// Select bus ID depending on the transport used
#ifdef MICRO_ROS_TRANSPORT_CAN
    status_msg.bus_id = __CONCAT(CAN_BUS_NAME, _ID);
#elif MICRO_ROS_TRANSPORT_ETH
    status_msg.bus_id = ETHERNET_BUS_ID;
#else
    status_msg.bus_id = 0;
#endif

    status_msg.client_id = client_id;
    status_msg.uptime_ms = to_ms_since_boot(get_absolute_time());
    status_msg.version_major = MAJOR_VERSION;
    status_msg.version_minor = MINOR_VERSION;
    status_msg.version_release_type = RELEASE_TYPE;
    status_msg.faults = *fault_list_reg;
    status_msg.kill_switches_enabled = 0;
    status_msg.kill_switches_asserting_kill = 0;
    status_msg.kill_switches_needs_update = 0;
    status_msg.kill_switches_timed_out = 0;

    for (int i = 0; i < NUM_KILL_SWITCHES; i++) {
        if (kill_switch_states[i].enabled) {
            status_msg.kill_switches_enabled |= (1 << i);
        }

        if (kill_switch_states[i].asserting_kill) {
            status_msg.kill_switches_asserting_kill |= (1 << i);
        }

        if (kill_switch_states[i].needs_update) {
            status_msg.kill_switches_needs_update |= (1 << i);
        }

        if (kill_switch_states[i].needs_update && time_reached(kill_switch_states[i].update_timeout)) {
            status_msg.kill_switches_timed_out |= (1 << i);
        }
    }

    RCSOFTRETCHECK(rcl_publish(&firmware_status_publisher, &status_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_heartbeat_pulse(uint8_t client_id) {
    std_msgs__msg__Int8 heartbeat_msg;
    heartbeat_msg.data = client_id;
    rcl_ret_t ret = rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL);
    if (ret != RCL_RET_OK) {
        failed_heartbeats++;

        if (failed_heartbeats > MAX_MISSSED_HEARTBEATS) {
            ros_connected = false;
        }
    }
    else {
        failed_heartbeats = 0;
    }

    RCSOFTRETCHECK(ret);

    return RCL_RET_OK;
}

static inline void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
    ts->tv_sec = time_nanos / 1000000000;
    ts->tv_nsec = time_nanos % 1000000000;
}

// TODO: Add in node specific tasks here
rcl_ret_t ros_update_depth_publisher() {
    // if (depth_reading_valid()) {
    //     struct timespec ts;
    //     nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
    //     depth_msg.header.stamp.sec = ts.tv_sec;
    //     depth_msg.header.stamp.nanosec = ts.tv_nsec;

    //     depth_msg.depth = -depth_read();
    //     RCSOFTRETCHECK(rcl_publish(&depth_publisher, &depth_msg, NULL));
    // }

    return RCL_RET_OK;
}

rcl_ret_t ros_pressure_pub(float pressure) {
    std_msgs__msg__Float32 pressure_msg;
    pressure_msg.data = pressure;

    RCSOFTRETCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));

    return RCL_RET_OK;
}

// ========================================
// ROS Core
// ========================================

rcl_ret_t ros_init() {
    // ROS Core Initialization
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCRETCHECK(rclc_node_init_default(&node, PICO_TARGET_NAME, ROBOT_NAMESPACE, &support));

    // Node Initialization
    RCRETCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&firmware_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareStatus),
                                           FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(
        &left_power_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), LEFT_POWER_SUBSCRIBER_NAME));
    RCRETCHECK(rclc_subscription_init_default(
        &right_power_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), RIGHT_POWER_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &pressure_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), PRESSURE_PUBLISHER_NAME));

    RCRETCHECK(rclc_service_init_default(&flush_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         FLUSH_SERVICE_NAME));

    // RCRETCHECK(rclc_publisher_init(&depth_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth),
    //                                DEPTH_PUBLISHER_NAME, &rmw_qos_profile_sensor_data));

    // Executor Initialization
    const int executor_num_handles = 4;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &left_power_subscriber, &left_power_msg,
                                              &left_power_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &right_power_subscriber, &right_power_msg,
                                              &right_power_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_service(&executor, &flush_service, &flush_service_req, &flush_service_res,
                                         &flush_service_callback));

    // Note: Code in executor callbacks should be kept to a minimum
    // It should set whatever flags are necessary and get out
    // And it should *NOT* try to perform any communiations over ROS, as this can lead to watchdog timeouts
    // in the event that specific request times out

    depth_msg.header.frame_id.data = depth_frame;
    depth_msg.header.frame_id.capacity = sizeof(depth_frame);
    depth_msg.header.frame_id.size = strlen(depth_frame);
    depth_msg.variance = depth_variance;

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    // TODO: Modify to clean up anything you have opened in init here to avoid memory leaks

    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&left_power_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&right_power_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pressure_publisher, &node));
    RCSOFTCHECK(rcl_service_fini(&flush_service, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    RCSOFTCHECK(rcl_publisher_fini(&depth_publisher, &node));

    ros_connected = false;
}

bool is_ros_connected(void) {
    return ros_connected;
}

bool ros_ping(void) {
    ros_connected = rmw_uros_ping_agent(RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT, 1) == RCL_RET_OK;
    return ros_connected;
}
