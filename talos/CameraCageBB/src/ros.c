#include "pico/stdlib.h"
#include <time.h>

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/led_command.h>
#include <riptide_msgs2/msg/depth.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include "driver/status_strip.h"
#include "titan/logger.h"
#include "titan/version.h"

#include "depth_sensor.h"
#include "ros.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define DEPTH_PUBLISHER_NAME "state/depth/raw"
#define WATER_TEMP_PUBLISHER_NAME "state/depth/temp"
#define LED_SUBSCRIBER_NAME "command/led"
#define PHYSICAL_KILL_NOTIFY_SUBSCRIBER_NAME "state/physkill_notify"
#define ELECTRICAL_COMMAND_SUBSCRIBER_NAME "command/electrical"

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

static alarm_id_t toggle_pwr_alarm_id = 0;

static int64_t set_computer_power(__unused alarm_id_t alarm, void * data){
    bool state = (bool) data;
    gpio_put(ORIN_SW_PIN, state);

    if (state) {
        toggle_pwr_alarm_id = 0;
        return 0; // Don't reschedule if turning on
    } else {
        return 1000 * 1000;   // Turn off for 1 second
    }
}

static void elec_command_subscription_callback(const void * msgin){
    const riptide_msgs2__msg__ElectricalCommand * msg = (const riptide_msgs2__msg__ElectricalCommand *)msgin;
    if (msg->command == riptide_msgs2__msg__ElectricalCommand__CYCLE_COMPUTER) {
        if (toggle_pwr_alarm_id != 0) {
            LOG_WARN("Attempting to request multiple cycles in a row");
        }
        else {
            toggle_pwr_alarm_id = add_alarm_in_ms(10, &set_computer_power, (void *) false,  true);
            if (toggle_pwr_alarm_id < 0) {
                toggle_pwr_alarm_id = 0;
                LOG_WARN("Failed to schedule set computer power alarm");
                safety_raise_fault(FAULT_ROS_ERROR);
            }
        }
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__ENABLE_FANS) {
        gpio_put(FAN_SW_PIN, true);
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__DISABLE_FANS) {
        gpio_put(FAN_SW_PIN, false);
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__ENABLE_LEDS) {
        status_strip_enable();
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__DISABLE_LEDS) {
        status_strip_disable();
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__CLEAR_DEPTH) {
        depth_recalibrate();
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
            status_msg.kill_switches_enabled |= (1<<i);
        }

        if (kill_switch_states[i].asserting_kill) {
            status_msg.kill_switches_asserting_kill |= (1<<i);
        }

        if (kill_switch_states[i].needs_update) {
            status_msg.kill_switches_needs_update |= (1<<i);
        }

        if (kill_switch_states[i].needs_update && time_reached(kill_switch_states[i].update_timeout)) {
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
    RCRETCHECK(rclc_node_init_default(&node, PICO_TARGET_NAME, ROBOT_NAMESPACE, &support));

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

    RCRETCHECK(rclc_subscription_init_default(
        &elec_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalCommand),
        ELECTRICAL_COMMAND_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 4;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_command_msg, &led_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &physkill_notify_subscriber, &physkill_notify_msg, &physkill_notify_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &elec_command_subscriber, &elec_command_msg, &elec_command_subscription_callback, ON_NEW_DATA));

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
    RCSOFTCHECK(rcl_subscription_fini(&elec_command_subscriber, &node));
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

