#include "ros.h"

// #include "ledc.h"
#include "ledc_driver.h"

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
#include <riptide_msgs2/msg/led_command.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define LED_SUBSCRIBER_NAME "command/led"
#define PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME "state/physkill_notify"
#define DEPTH_SUBSCRIBER_NAME "state/depth/raw"

bool ros_connected = false;
bool pulse = false;

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

rcl_subscription_t led_subscriber;
riptide_msgs2__msg__LedCommand led_command_msg;
rcl_subscription_t physkill_notify_subscriber;
std_msgs__msg__Bool physkill_notify_msg;
rcl_subscription_t depth_subscriber;
riptide_msgs2__msg__Depth depth_msg;

// TODO: Add node specific items here

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void led_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__LedCommand *msg = (const riptide_msgs2__msg__LedCommand *) msgin;

    // Ignore commands not for us
    if ((msg->target & riptide_msgs2__msg__LedCommand__TARGET_ALU) == 0) {
        return;
    }

    enum status_mode mode;
    bool is_singleton = false;

    // Convert message mode to local mode
    switch (msg->mode) {
    case riptide_msgs2__msg__LedCommand__MODE_SOLID:
        mode = MODE_SOLID;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_SLOW_FLASH:
        mode = MODE_SLOW_FLASH;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_FAST_FLASH:
        mode = MODE_FAST_FLASH;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_BREATH:
        mode = MODE_BREATH;
        break;
    case riptide_msgs2__msg__LedCommand__SINGLETON_FLASH:
        is_singleton = true;
        break;
    default:
        led_clear();
        return;
    }

    if (is_singleton)
        led_singleton(msg->red, msg->green, msg->blue);
    else
        led_set(mode, msg->red, msg->green, msg->blue);
}

static void physkill_notify_subscription_callback(const void *msgin) {
    // This topic is published whenever the physical killswitch topic is published
    // This will flash the LED strip for feedback that the kill switch is inserted
    // This is required as the thrusters won't chime if software kill is active, so this is the only source of
    // feedback for whoever inserts the kill switch

    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    if (msg->data) {  // Note true means that the physical kill switch is asserted (switch removed)
        // Flash red on kill switch removal
        led_flash(255, 0, 0);
    }
    else {
        // Flash blue on kill switch insertion
        led_flash(0, 0, 255);
    }
}

static void depth_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__Depth *msg = (const riptide_msgs2__msg__Depth *) msgin;
    led_depth_set(msg->depth);
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

// TODO: Add in node specific tasks here

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
        &led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, LedCommand), LED_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&physkill_notify_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                              PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME));
    RCRETCHECK(rclc_subscription_init_default(
        &depth_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth), DEPTH_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 4;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_command_msg, &led_subscription_callback,
                                              ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &physkill_notify_subscriber, &physkill_notify_msg,
                                              &physkill_notify_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &depth_subscriber, &depth_msg, &depth_subscription_callback,
                                              ON_NEW_DATA));

    // TODO: Modify this method with node specific objects

    // Note: Code in executor callbacks should be kept to a minimum
    // It should set whatever flags are necessary and get out
    // And it should *NOT* try to perform any communiations over ROS, as this can lead to watchdog timeouts
    // in the event that specific request times out

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    // TODO: Modify to clean up anything you have opened in init here to avoid memory leaks

    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&led_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&physkill_notify_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
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
