#include "pico/stdlib.h"

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <riptide_msgs2/msg/dshot_command.h>
#include <riptide_msgs2/msg/dshot_partial_telemetry.h>
#include <riptide_msgs2/msg/dshot_rpm_feedback.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

#include "build_version.h"
#include "basic_logger/logging.h"

#include "dshot.h"
#include "ros.h"

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
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define DSHOT_COMMAND_SUCRIBER_NAME "command/dshot"
#define DSHOT_RPM_PUBLISHER_NAME "state/thrusters/rpm"
#define DSHOT_TELEMETRY_PUBLISHER_NAME "state/thrusters/telemetry"

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
rcl_subscription_t killswtich_subscriber;
std_msgs__msg__Bool killswitch_msg;

rcl_publisher_t dshot_telem_publisher;
rcl_publisher_t dshot_rpm_publisher;
rcl_subscription_t dshot_subscriber;
riptide_msgs2__msg__DshotCommand dshot_msg;

// ========================================
// ROS Callbacks
// ========================================

static void killswitch_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void dshot_subscription_callback(const void * msgin) {
    const riptide_msgs2__msg__DshotCommand * msg = (const riptide_msgs2__msg__DshotCommand *)msgin;
    extern uint8_t esc_board_num;
    uint offset = (esc_board_num == 1 ? 4 : 0);
    dshot_update_thrusters(&msg->values[offset]);
    dshot_command_received = true;
}

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

rcl_ret_t ros_send_rpm(uint8_t board_id) {
    riptide_msgs2__msg__DshotRPMFeedback rpm_msg;
    uint8_t valid_mask = 0;
    uint thruster_base = (board_id == 0 ? 0 : 4);

    for (int i = 0; i < NUM_THRUSTERS; i++) {
        int16_t rpm = 0;

        if (dshot_rpm_data[i].valid) {
            valid_mask |= 1<<(thruster_base + i);

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

        rpm_msg.rpm[thruster_base + i] = rpm;
    }

    rpm_msg.rpm_valid_mask = valid_mask;

    RCSOFTRETCHECK(rcl_publish(&dshot_rpm_publisher, &rpm_msg, NULL));

    return RCL_RET_OK;
}


rcl_ret_t ros_send_telemetry(uint8_t board_id) {
    riptide_msgs2__msg__DshotPartialTelemetry telem_msg;
    telem_msg.vcc_voltage = (vcc_reading_mv/1000.f);
    telem_msg.escs_powered = esc_board_on;
    telem_msg.start_thruster_num = (board_id == 0 ? 0 : 4);

    for (int i = 0; i < NUM_THRUSTERS; i++) {
        if (dshot_telemetry_data[i].valid) {
            telem_msg.esc_telemetry[i].present = true;
            telem_msg.esc_telemetry[i].temperature_c = dshot_telemetry_data[i].temperature;
            telem_msg.esc_telemetry[i].voltage = dshot_telemetry_data[i].voltage / 100.0;
            telem_msg.esc_telemetry[i].current = dshot_telemetry_data[i].current / 100.0;
            telem_msg.esc_telemetry[i].consumption_ah = dshot_telemetry_data[i].consumption / 1000.0;
            telem_msg.esc_telemetry[i].rpm = (dshot_rpm_reversed[i] ? -1 : 1) * dshot_telemetry_data[i].rpm * 100 / num_pole_pairs;
        } else {
            telem_msg.esc_telemetry[i].present = false;
            telem_msg.esc_telemetry[i].temperature_c = 0;
            telem_msg.esc_telemetry[i].voltage = 0;
            telem_msg.esc_telemetry[i].current = 0;
            telem_msg.esc_telemetry[i].consumption_ah = 0;
            telem_msg.esc_telemetry[i].rpm = 0;
        }
    }

    RCSOFTRETCHECK(rcl_publish(&dshot_telem_publisher, &telem_msg, NULL));

    return RCL_RET_OK;
}


// ========================================
// ROS Core
// ========================================

rcl_ret_t ros_init(uint8_t board_id) {
    // ROS Core Initialization
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    const char *node_name = (board_id == 0 ? PICO_BOARD "_0_firmware" : PICO_BOARD "_1_firmware");
    RCRETCHECK(rclc_node_init_default(&node, node_name, ROBOT_NAMESPACE, &support));

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

    RCRETCHECK(rclc_publisher_init_best_effort(
        &dshot_telem_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotPartialTelemetry),
        DSHOT_TELEMETRY_PUBLISHER_NAME));

    // Executor Initialization
    const int executor_num_handles = 2;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));

    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_subscriber, &dshot_msg, &dshot_subscription_callback, ON_NEW_DATA));

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    RCSOFTCHECK(rcl_publisher_fini(&dshot_rpm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&dshot_telem_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_subscriber, &node));
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

