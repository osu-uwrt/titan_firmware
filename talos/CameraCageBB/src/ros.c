#include "pico/stdlib.h"

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/led_command.h>
#include <riptide_msgs2/msg/depth.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include "build_version.h"
#include "basic_logger/logging.h"

#include "depth_sensor.h"
#include "ros.h"
#include "status_strip.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// RMW Error Handling Code
// ========================================

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

#define lookup_string_enum(value, list) ((value < sizeof(list)/sizeof(*list)) ? list[value] : "Out-of-Bounds")
#define lookup_entity_enum(value) lookup_string_enum(value, entity_lookup_table)
#define lookup_source_enum(value) lookup_string_enum(value, source_lookup_table)

void rmw_error_cb(
  __unused const rmw_uros_error_entity_type_t entity,
  __unused const rmw_uros_error_source_t source,
  __unused const rmw_uros_error_context_t context,
  __unused const char * file,
  __unused const int line) {
    LOG_DEBUG("RMW UROS Error:\n\tEntity: %s\n\tSource: %s\n\tDesc: %s\n\tLocation: %s:%d", lookup_entity_enum(entity), lookup_source_enum(source), context.description, file, line);
}

void ros_rmw_init_error_handling(void)  {
    rmw_uros_set_error_handling_callback(rmw_error_cb);
}

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define DEPTH_PUBLISHER_NAME "depth/raw"
#define WATER_TEMP_PUBLISHER_NAME "depth/temp"
#define LED_SUBSCRIBER_NAME "command/led"
#define PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME "state/physkill_notify"

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
rcl_subscription_t led_subscriber;
riptide_msgs2__msg__LedCommand led_command_msg;
rcl_subscription_t physkill_notify_subscriber;
std_msgs__msg__Bool physkill_notify_msg;

// Depth Sensor
rcl_publisher_t depth_publisher;
rcl_publisher_t water_temp_publisher;
riptide_msgs2__msg__Depth depth_msg;
char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
const float depth_variance = 0.003;

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void led_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__LedCommand * msg = (const riptide_msgs2__msg__LedCommand *)msgin;

    // Ignore commands not for us
    if ((msg->target & riptide_msgs2__msg__LedCommand__TARGET_CCB) == 0) {
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

static void physkill_notify_subscription_callback(const void * msgin)
{
    // This topic is published whenever the physical killswitch topic is published
    // This will flash the front of the LED strip for feedback that the kill switch is inserted
    // This is required as the thrusters won't chime if software kill is active, so this is the only source of
    // feedback for whoever inserts the kill switch

	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {    // Note true means that the physical kill switch is asserted (switch removed)
        // Flash red on kill switch removal
        status_strip_flash_front(255, 0, 0);
    }
    else {
        // Flash blue on kill switch insertion
        status_strip_flash_front(0, 0, 255);
    }
}

// ========================================
// Public Task Methods (called in main tick)
// ========================================

rcl_ret_t ros_update_firmware_status(uint8_t client_id) {
    riptide_msgs2__msg__FirmwareStatus status_msg;
    status_msg.board_name.data = PICO_BOARD;
    status_msg.board_name.size = strlen(PICO_BOARD);
    status_msg.board_name.capacity = status_msg.board_name.size + 1; // includes NULL byte
    status_msg.client_id = client_id;
    status_msg.uptime_ms = to_ms_since_boot(get_absolute_time());
    status_msg.version_major = MAJOR_VERSION;
    status_msg.version_minor = MINOR_VERSION;
    status_msg.faults = *fault_list_reg;
    status_msg.kill_switches_enabled = 0;
    status_msg.kill_switches_asserting_kill = 0;
    status_msg.kill_switches_needs_update = 0;
    status_msg.kill_switches_timed_out = 0;

    absolute_time_t now = get_absolute_time();
    for (int i = 0; i < NUM_KILL_SWITCHES; i++) {
        if (kill_switch_states[i].enabled) {
            status_msg.kill_switches_enabled |= (1<<i);
        }

        if (kill_switch_states[i].asserting_kill) {
            status_msg.kill_switches_asserting_kill |= (1<<i);
        }

        if (kill_switch_states[i].needs_update) {
            status_msg.kill_switches_needs_update |= (1<<i);
        }

        if (kill_switch_states[i].needs_update && absolute_time_diff_us(now, kill_switch_states[i].update_timeout) < 0) {
            status_msg.kill_switches_timed_out |= (1<<i);
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

        if(failed_heartbeats > MAX_MISSSED_HEARTBEATS) {
            ros_connected = false;
        }
    } else {
        failed_heartbeats = 0;
    }

    RCSOFTRETCHECK(ret);

    return RCL_RET_OK;
}

static inline void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
	ts->tv_sec = time_nanos / 1000000000;
	ts->tv_nsec = time_nanos % 1000000000;
}

rcl_ret_t ros_update_depth_publisher() {
    if (depth_reading_valid()) {
		struct timespec ts;
		nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
		depth_msg.header.stamp.sec = ts.tv_sec;
		depth_msg.header.stamp.nanosec = ts.tv_nsec;

		depth_msg.depth = -depth_read();
		RCSOFTRETCHECK(rcl_publish(&depth_publisher, &depth_msg, NULL));
	}

    return RCL_RET_OK;
}

rcl_ret_t ros_update_water_temp_publisher() {
    if (depth_reading_valid()) {
        std_msgs__msg__Float32 water_temp_msg;
		water_temp_msg.data = depth_get_temperature();
		RCSOFTRETCHECK(rcl_publish(&water_temp_publisher, &water_temp_msg, NULL));
	}

    return RCL_RET_OK;
}

// ========================================
// ROS Core
// ========================================

rcl_ret_t ros_init() {
    // ROS Core Initialization
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCRETCHECK(rclc_node_init_default(&node, PICO_BOARD "_firmware", ROBOT_NAMESPACE, &support));

    // Node Initialization
    RCRETCHECK(rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &firmware_status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareStatus),
        FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_publisher_init(
		&depth_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth),
		DEPTH_PUBLISHER_NAME,
		&rmw_qos_profile_sensor_data));

    RCRETCHECK(rclc_publisher_init(
		&water_temp_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		WATER_TEMP_PUBLISHER_NAME,
		&rmw_qos_profile_sensor_data));

    RCRETCHECK(rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, LedCommand),
        LED_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(
        &physkill_notify_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 3;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_command_msg, &led_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &physkill_notify_subscriber, &physkill_notify_msg, &physkill_notify_subscription_callback, ON_NEW_DATA));

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
    RCSOFTCHECK(rcl_subscription_fini(&physkill_notify_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&led_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&water_temp_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&depth_publisher, &node));
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

