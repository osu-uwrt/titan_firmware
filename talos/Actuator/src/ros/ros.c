#include "ros.h"

#include "actuators.h"
#include "ros_internal.h"

#include "driver/status_strip.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/actuator_status.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/led_command.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>

// ========================================
// Global Definitions
// ========================================

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

// Electrical System
rcl_subscription_t led_subscriber;
riptide_msgs2__msg__LedCommand led_command_msg;
rcl_subscription_t physkill_notify_subscriber;
std_msgs__msg__Bool physkill_notify_msg;
rcl_subscription_t elec_command_subscriber;
riptide_msgs2__msg__ElectricalCommand elec_command_msg;

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
    if ((msg->target & riptide_msgs2__msg__LedCommand__TARGET_ACT) == 0) {
        return;
    }

    enum status_strip_mode mode;

    // Convert message mode to local mode
    switch (msg->mode) {
    case riptide_msgs2__msg__LedCommand__MODE_SOLID:
        mode = STATUS_STRIP_MODE_SOLID;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_SLOW_FLASH:
        mode = STATUS_STRIP_MODE_SLOW_FLASH;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_FAST_FLASH:
        mode = STATUS_STRIP_MODE_FAST_FLASH;
        break;
    case riptide_msgs2__msg__LedCommand__MODE_BREATH:
        mode = STATUS_STRIP_MODE_BREATH;
        break;
    default:
        status_strip_clear();
        return;
    }

    status_strip_set(mode, msg->red, msg->green, msg->blue);
}

static void physkill_notify_subscription_callback(const void *msgin) {
    // This topic is published whenever the physical killswitch topic is published
    // This will flash the LED strip for feedback that the kill switch is inserted
    // This is required as the thrusters won't chime if software kill is active, so this is the only source of
    // feedback for whoever inserts the kill switch

    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    if (msg->data) {  // Note true means that the physical kill switch is asserted (switch removed)
        // Flash red on kill switch removal
        status_strip_status_flash(255, 0, 0);
    }
    else {
        // Flash blue on kill switch insertion
        status_strip_status_flash(0, 0, 255);
    }
}

static void elec_command_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__ElectricalCommand *msg = (const riptide_msgs2__msg__ElectricalCommand *) msgin;
    if (msg->command == riptide_msgs2__msg__ElectricalCommand__ENABLE_LEDS) {
        status_strip_enable();
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__DISABLE_LEDS) {
        status_strip_disable();
    }
}

// ========================================
// Public Task Methods (called in main tick)
// ========================================

rcl_ret_t ros_update_firmware_status(uint8_t client_id) {
    riptide_msgs2__msg__FirmwareStatus status_msg;
    status_msg.board_name.data = PICO_BOARD;
    status_msg.board_name.size = strlen(PICO_BOARD);
    status_msg.board_name.capacity = status_msg.board_name.size + 1;  // includes NULL byte
    status_msg.bus_id = __CONCAT(CAN_BUS_NAME, _ID);
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

    RCRETCHECK(rclc_subscription_init_default(&elec_command_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalCommand),
                                              ELECTRICAL_COMMAND_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_local_handles = 4;
#if ACTUATOR_V1_SUPPORT
    const int executor_num_handles =
        ros_actuators_num_executor_handles + actuator_v1_parameters_num_executor_handles + executor_num_local_handles;
#elif ACTUATOR_V2_SUPPORT
    const int executor_num_handles =
        ros_actuators_num_executor_handles + actuator_v2_dynamixel_num_executor_handles + executor_num_local_handles;
#else
#error Unexpected Actuator Support
#endif

    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_command_msg, &led_subscription_callback,
                                              ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &physkill_notify_subscriber, &physkill_notify_msg,
                                              &physkill_notify_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &elec_command_subscriber, &elec_command_msg,
                                              &elec_command_subscription_callback, ON_NEW_DATA));

    // Initialize other files
    RCRETCHECK(ros_actuators_init(&executor, &node));

#if ACTUATOR_V1_SUPPORT
    RCRETCHECK(actuator_v1_parameters_init(&node, &executor));
#endif

#if ACTUATOR_V2_SUPPORT
    RCRETCHECK(actuator_v2_dynamixel_init(&node, &executor));
#endif

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
#if ACTUATOR_V1_SUPPORT
    actuator_v1_parameters_fini(&node);
#endif

#if ACTUATOR_V2_SUPPORT
    actuator_v2_dynamixel_fini(&node);
#endif

    ros_actuators_fini(&node);

    RCSOFTCHECK(rcl_subscription_fini(&elec_command_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&physkill_notify_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&led_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node))
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
