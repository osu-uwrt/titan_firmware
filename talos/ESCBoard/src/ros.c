#include "ros.h"

#include "core1.h"
#include "dshot.h"

#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/dshot_command.h>
#include <riptide_msgs2/msg/dshot_partial_telemetry.h>
#include <riptide_msgs2/msg/dshot_rpm_feedback.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define DSHOT_COMMAND_SUCRIBER_NAME "command/thruster_rpm"
#define DSHOT_RPM_PUBLISHER_NAME "state/thrusters/rpm"
#define DSHOT_CMD_PUBLISHER_NAME "state/thrusters/cmd"
#define DSHOT_TELEMETRY_PUBLISHER_NAME "state/thrusters/telemetry"

#define DSHOT_TUNE_SET_RAW "dshot_tune/set_raw"
#define DSHOT_TUNE_P_GAIN "dshot_tune/p_gain"
#define DSHOT_TUNE_I_GAIN "dshot_tune/i_gain"
#define DSHOT_TUNE_I_BOUND "dshot_tune/i_bound"
#define DSHOT_TUNE_HARD_LIMIT "dshot_tune/hard_limit"
#define DSHOT_TUNE_MIN_COMMAND "dshot_tune/min_command"

bool ros_connected = false;
bool dshot_command_received = false;

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
rcl_publisher_t dshot_cmd_publisher;
rcl_subscription_t dshot_subscriber;
rcl_subscription_t dshot_tune_set_raw_subscriber;
rcl_subscription_t dshot_tune_p_gain_subscriber;
rcl_subscription_t dshot_tune_i_gain_subscriber;
rcl_subscription_t dshot_tune_i_bound_subscriber;
rcl_subscription_t dshot_tune_hard_limit_subscriber;
rcl_subscription_t dshot_tune_min_command_subscriber;
riptide_msgs2__msg__DshotCommand dshot_msg;
std_msgs__msg__Bool dshot_tune_set_raw_msg;
std_msgs__msg__Int32 dshot_tune_p_gain_msg;
std_msgs__msg__Int32 dshot_tune_i_gain_msg;
std_msgs__msg__Int32 dshot_tune_i_bound_msg;
std_msgs__msg__Int32 dshot_tune_hard_limit_msg;
std_msgs__msg__Int32 dshot_tune_min_command_msg;

// ========================================
// ROS Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void dshot_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__DshotCommand *msg = (const riptide_msgs2__msg__DshotCommand *) msgin;
    extern uint8_t esc_board_num;
    uint offset = (esc_board_num == 1 ? 4 : 0);
    core1_update_target_rpm(&msg->values[offset]);
    dshot_command_received = true;
}

static void dshot_tune_set_raw_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    core1_set_raw_mode(msg->data);
}

static void dshot_tune_p_gain_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    core1_set_p_gain(msg->data);
}

static void dshot_tune_i_gain_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    core1_set_i_gain(msg->data);
}

static void dshot_tune_i_bound_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    core1_set_i_bound(msg->data);
}

static void dshot_tune_hard_limit_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    core1_set_hard_limit(msg->data);
}

static void dshot_tune_min_command_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    core1_set_min_command(msg->data);
}

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

rcl_ret_t ros_send_rpm(uint8_t board_id) {
    riptide_msgs2__msg__DshotRPMFeedback rpm_msg = { 0 };
    riptide_msgs2__msg__DshotRPMFeedback cmd_msg = { 0 };
    uint8_t valid_mask = 0;
    uint thruster_base = (board_id == 0 ? 0 : 4);

    struct core1_telem telem_data;
    core1_get_telem(&telem_data, false);

    for (int i = 0; i < NUM_THRUSTERS; i++) {
        if (telem_data.thruster[i].rpm_valid) {
            valid_mask |= 1 << (thruster_base + i);
            rpm_msg.rpm[thruster_base + i] = telem_data.thruster[i].rpm;
        }

        cmd_msg.rpm[thruster_base + i] = telem_data.thruster[i].cmd;
    }

    rpm_msg.rpm_valid_mask = valid_mask;
    cmd_msg.rpm_valid_mask = valid_mask;

    RCSOFTRETCHECK(rcl_publish(&dshot_rpm_publisher, &rpm_msg, NULL));
    RCSOFTRETCHECK(rcl_publish(&dshot_cmd_publisher, &cmd_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_send_telemetry(uint8_t board_id) {
    riptide_msgs2__msg__DshotPartialTelemetry telem_msg;

    // Capture the telemetry from core1
    struct core1_telem core1_telem_data;
    core1_get_telem(&core1_telem_data, true);

    // Copy in all global telemetry data
    telem_msg.vcc_voltage = (vcc_reading_mv / 1000.f);
    telem_msg.escs_powered = esc_board_on;
    telem_msg.thrusters_moving = dshot_thrusters_on;
    telem_msg.start_thruster_num = (board_id == 0 ? 0 : 4);
    telem_msg.disabled_flags = core1_telem_data.disabled_flags;
    telem_msg.packet_delta_us = core1_telem_data.time_delta_us;
    telem_msg.tick_cnt = core1_telem_data.controller_tick_cnt;

    // Copy in thruster-specific telemtry data
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        struct dshot_uart_telemetry uart_telem_data;
        if (dshot_get_telemetry(i, &uart_telem_data)) {
            // ESC is only online if we have both RPM and telemtry coming out of it
            telem_msg.esc_telemetry[i].esc_online = core1_telem_data.thruster[i].rpm_valid;

            // Copy in the uart telemetry data
            telem_msg.esc_telemetry[i].temperature_c = uart_telem_data.temperature;
            telem_msg.esc_telemetry[i].voltage = uart_telem_data.voltage / 100.0;
            telem_msg.esc_telemetry[i].current = uart_telem_data.current / 100.0;
            telem_msg.esc_telemetry[i].consumption_ah = uart_telem_data.consumption / 1000.0;
            telem_msg.esc_telemetry[i].rpm =
                (uart_telem_data.rpm_reversed ? -1 : 1) * uart_telem_data.rpm * 100 / ESC_NUM_POLE_PAIRS;
        }
        else {
            telem_msg.esc_telemetry[i].esc_online = false;
            telem_msg.esc_telemetry[i].temperature_c = 0;
            telem_msg.esc_telemetry[i].voltage = 0;
            telem_msg.esc_telemetry[i].current = 0;
            telem_msg.esc_telemetry[i].consumption_ah = 0;
            telem_msg.esc_telemetry[i].rpm = 0;
        }

        // Copy in uart profiling data
        telem_msg.esc_telemetry[i].uart_disabled_cnt = uart_telem_data.total_invalid;
        telem_msg.esc_telemetry[i].uart_missed_cnt = uart_telem_data.total_missed;
        telem_msg.esc_telemetry[i].uart_success_cnt = uart_telem_data.total_success;

        // Copy in profiling data from control loop
        telem_msg.esc_telemetry[i].thruster_ready = core1_telem_data.thruster[i].thruster_ready;
        telem_msg.esc_telemetry[i].dshot_missed_cnt = core1_telem_data.thruster[i].ticks_missed;
        telem_msg.esc_telemetry[i].dshot_offline_cnt = core1_telem_data.thruster[i].ticks_offline;
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

    const char *node_name = (board_id == 0 ? PICO_TARGET_NAME "_0" : PICO_TARGET_NAME "_1");
    RCRETCHECK(rclc_node_init_default(&node, node_name, ROBOT_NAMESPACE, &support));

    // Node Initialization
    RCRETCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&firmware_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareStatus),
                                           FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(&dshot_subscriber, &node,
                                                  ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotCommand),
                                                  DSHOT_COMMAND_SUCRIBER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&dshot_rpm_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotRPMFeedback),
                                               DSHOT_RPM_PUBLISHER_NAME));
    RCRETCHECK(rclc_publisher_init_best_effort(&dshot_cmd_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotRPMFeedback),
                                               DSHOT_CMD_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&dshot_telem_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DshotPartialTelemetry),
                                               DSHOT_TELEMETRY_PUBLISHER_NAME));

    // Dshot tuning subscribers
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_set_raw_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), DSHOT_TUNE_SET_RAW));
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_p_gain_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), DSHOT_TUNE_P_GAIN));
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_i_gain_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), DSHOT_TUNE_I_GAIN));
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_i_bound_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), DSHOT_TUNE_I_BOUND));
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_hard_limit_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                              DSHOT_TUNE_HARD_LIMIT));
    RCRETCHECK(rclc_subscription_init_default(&dshot_tune_min_command_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                              DSHOT_TUNE_MIN_COMMAND));

    // Executor Initialization
    const int executor_num_handles = 8;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));

    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_subscriber, &dshot_msg, &dshot_subscription_callback,
                                              ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_set_raw_subscriber, &dshot_tune_set_raw_msg,
                                              &dshot_tune_set_raw_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_p_gain_subscriber, &dshot_tune_p_gain_msg,
                                              &dshot_tune_p_gain_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_i_gain_subscriber, &dshot_tune_i_gain_msg,
                                              &dshot_tune_i_gain_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_i_bound_subscriber, &dshot_tune_i_bound_msg,
                                              &dshot_tune_i_bound_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_hard_limit_subscriber, &dshot_tune_hard_limit_msg,
                                              &dshot_tune_hard_limit_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &dshot_tune_min_command_subscriber,
                                              &dshot_tune_min_command_msg,
                                              &dshot_tune_min_command_subscription_callback, ON_NEW_DATA));

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    RCSOFTCHECK(rcl_publisher_fini(&dshot_rpm_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&dshot_cmd_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&dshot_telem_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_set_raw_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_p_gain_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_i_gain_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_i_bound_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_hard_limit_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&dshot_tune_min_command_subscriber, &node));
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
