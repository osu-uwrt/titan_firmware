#include "pico/stdlib.h"

#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <riptide_msgs2/msg/depth.h>
#include <riptide_msgs2/msg/dshot_command.h>
#include <riptide_msgs2/msg/dshot_rpm_feedback.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <riptide_msgs2/msg/electrical_readings.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>


#include "build_version.h"
#include "basic_logger/logging.h"

#include "mcp3426.h"
#include "depth_sensor.h"
#include "dshot.h"
#include "ros.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

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
#define KILLSWITCH_PUBLISHER_NAME "state/kill"
#define DSHOT_COMMAND_SUCRIBER_NAME "command/dshot"
#define DSHOT_RPM_PUBLISHER_NAME "thrusters/rpm"
#define SOFTWARE_KILL_PUBLISHER_NAME "control/software_kill"
#define DEPTH_PUBLISHER_NAME "depth/raw"
#define WATER_TEMP_PUBLISHER_NAME "depth/temp"
#define ADC_PUBLISHER_NAME "adc/voltage"

bool ros_connected = false;
bool dshot_command_received = false;

// Constant for converting electrical RPM to mechanical RPM
// Found on forum: https://discuss.bluerobotics.com/t/t200-thruster-questions-poles-max-voltage-e-bike-controller/2442/2
const int num_pole_pairs = 7;

// Core Variables
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t heartbeat_publisher;
int failed_heartbeats = 0;

// Node specific Variables
rcl_publisher_t firmware_status_publisher;

// Dshot publishers
rcl_publisher_t dshot_rpm_publisher;
rcl_subscription_t dshot_subscriber;
riptide_msgs2__msg__DshotCommand dshot_msg;

// Physical kill switch
rcl_publisher_t killswitch_publisher;
rcl_subscription_t software_kill_subscriber;
riptide_msgs2__msg__KillSwitchReport software_kill_msg;
char software_kill_frame_str[SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE+1] = {0};

// Depth Sensor
rcl_publisher_t depth_publisher;
rcl_publisher_t water_temp_publisher;
riptide_msgs2__msg__Depth depth_msg;
char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
const float depth_variance = 0.003;

// Voltage ADC
rcl_publisher_t adc_publisher;
riptide_msgs2__msg__ElectricalReadings status_msg = {0};

// ========================================
// Executor Callbacks
// ========================================

static void dshot_subscription_callback(const void * msgin) {
    const riptide_msgs2__msg__DshotCommand * msg = (const riptide_msgs2__msg__DshotCommand *)msgin;
    dshot_update_thrusters(msg->values);
    // dshot_command_received = true;
}

static void software_kill_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__KillSwitchReport * msg = (const riptide_msgs2__msg__KillSwitchReport *)msgin;

    // Make sure kill switch id is valid
    if (msg->kill_switch_id >= riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES ||
            msg->kill_switch_id == riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL) {
        LOG_WARN("Invalid kill switch id used %d", msg->kill_switch_id);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Make sure frame id isn't too large
    if (msg->sender_id.size >= SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE) {
        LOG_WARN("Software Kill Frame ID too large");
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    struct kill_switch_state* kill_entry = &kill_switch_states[msg->kill_switch_id];

    if (kill_entry->enabled && kill_entry->asserting_kill && !msg->switch_asserting_kill &&
            strncmp(kill_entry->locking_frame, msg->sender_id.data, SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE)) {
        LOG_WARN("Invalid frame ID to unlock kill switch %d ('%s' expected, '%s' requested)", msg->kill_switch_id, kill_entry->locking_frame, msg->sender_id.data);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Set frame id of the switch requesting disable
    // This can technically override previous kills and allow one node to kill when another is stopping
    // However, this is mainly to prevent getting locked out if, for example, rqt crashed and the robot was killed
    // If multiple points are needed to be separate, separate kill switch IDs should be used
    // This will protect from someone unexpectedly unkilling the robot by publishing a non assert kill
    // since it must be asserted as killed by the node to take ownership of the lock
    if (msg->switch_asserting_kill) {
        strncpy(kill_entry->locking_frame, msg->sender_id.data, msg->sender_id.size);
        kill_entry->locking_frame[msg->sender_id.size] = '\0';
    }

    safety_kill_switch_update(msg->kill_switch_id, msg->switch_asserting_kill, msg->switch_needs_update);
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
    for (int i = 0; i < riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES; i++) {
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

rcl_ret_t ros_publish_killswitch(void) {
    std_msgs__msg__Bool killswitch_msg = {.data = safety_kill_get_asserting_kill()};

    RCSOFTRETCHECK(rcl_publish(&killswitch_publisher, &killswitch_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t adc_update(uint8_t client_id) {
    status_msg.three_volt_voltage = (mcp3426_query(0) * .0033067);
    status_msg.five_volt_voltage = mcp3426_query(1);
    status_msg.twelve_volt_voltage = (mcp3426_query(3) * .0078905);
    status_msg.balanced_voltage = (mcp3426_query(2) * .0154);
    RCSOFTRETCHECK(rcl_publish(&adc_publisher, &status_msg, NULL));
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

rcl_ret_t ros_send_rpm(void) {
    riptide_msgs2__msg__DshotRPMFeedback rpm_msg;
    uint8_t valid_mask = 0;

    for (int i = 0; i < NUM_THRUSTERS; i++) {
        int16_t rpm = 0;

        if (dshot_rpm_data[i].valid) {
            valid_mask |= 1<<i;

            rpm = INT16_MAX;
            unsigned int period_us = dshot_rpm_data[i].rpm_period_us;

            // Check if the period isn't reporting not moving
            if (period_us == (0x1FF << 0x7)) {
                rpm = 0;
            } else if (period_us != 0) {
                // Convert period to RPM if it's valid
                uint32_t erpm = (60 * 1000000) / period_us;
                uint32_t rpm_unsigned = erpm / num_pole_pairs;

                // Make sure it won't overflow
                if (rpm_unsigned < INT16_MAX) {
                    rpm = rpm_unsigned;
                }
            }

            // Get the RPM sign
            if (dshot_rpm_reversed[i]) {
                rpm *= -1;
            }
        }

        rpm_msg.rpm[i] = rpm;
    }

    rpm_msg.rpm_valid_mask = valid_mask;

    RCSOFTRETCHECK(rcl_publish(&dshot_rpm_publisher, &rpm_msg, NULL));

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

    RCRETCHECK(rclc_publisher_init_best_effort(
        &killswitch_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        KILLSWITCH_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &dshot_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotCommand),
        DSHOT_COMMAND_SUCRIBER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(
        &dshot_rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotRPMFeedback),
        DSHOT_RPM_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
		&software_kill_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, KillSwitchReport),
		SOFTWARE_KILL_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &adc_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalReadings),
        ADC_PUBLISHER_NAME));

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


    // Executor Initialization
    const int executor_num_handles = 2;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &software_kill_subscriber, &software_kill_msg, &software_kill_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_subscriber, &dshot_msg, &dshot_subscription_callback, ON_NEW_DATA));

    depth_msg.header.frame_id.data = depth_frame;
	depth_msg.header.frame_id.capacity = sizeof(depth_frame);
	depth_msg.header.frame_id.size = strlen(depth_frame);
	depth_msg.variance = depth_variance;

    software_kill_msg.sender_id.data = software_kill_frame_str;
	software_kill_msg.sender_id.capacity = sizeof(software_kill_frame_str);
	software_kill_msg.sender_id.size = 0;

    // TODO: Code in executor callbacks should be kept to a minimum
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
    RCSOFTCHECK(rcl_publisher_fini(&water_temp_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&depth_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&adc_publisher, &node))
    RCSOFTCHECK(rcl_publisher_fini(&dshot_rpm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&killswitch_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&software_kill_subscriber, &node));
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

