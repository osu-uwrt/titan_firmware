#include "ros.h"

#include "core1.h"

#include "driver/sht41.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/battery_status.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define BATTERY_STATUS_PUBLISHER_NAME "state/battery"
#define KILLSWITCH_SUBCRIBER_NAME "state/kill"
#define ELECTRICAL_COMMAND_SUBSCRIBER_NAME "command/electrical"
#define TEMP_STATUS_PUBLISHER_NAME "status/temp/smartbattery"
#define HUMIDITY_STATUS_PUBLISHER_NAME "status/humidity/smartbattery"
#define SWKILL_SUBSCRIBER_NAME "command/software_kill"
#define LEAK_SUBSCRIBER_NAME "state/leak"

bool ros_connected = false;
bool has_rviz = false;

// Core Variables
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t heartbeat_publisher;
int failed_heartbeats = 0;

// Node specific Variables
rcl_publisher_t firmware_status_publisher, battery_status_publisher, temp_status_publisher, humidity_status_publisher;
rcl_subscription_t killswtich_subscriber, electrical_command_subscriber, swkill_subscriber, leak_subscriber;
std_msgs__msg__Bool killswitch_msg;
riptide_msgs2__msg__ElectricalCommand electrical_command_msg;
riptide_msgs2__msg__KillSwitchReport swkill_msg;
std_msgs__msg__Bool leak_msg;

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

static void electrical_command_callback(const void *msgin) {
    const riptide_msgs2__msg__ElectricalCommand *msg = (const riptide_msgs2__msg__ElectricalCommand *) msgin;

    // check if the command was a power cycle
    if (msg->command == riptide_msgs2__msg__ElectricalCommand__CYCLE_ROBOT) {
        LOG_INFO("ROS Command requested robot power cycle... Issuing temporary Emergency FET Shutdown");
        core1_power_cycle_robot();
    }

    // also check for a reset
    if (msg->command == riptide_msgs2__msg__ElectricalCommand__KILL_ROBOT_POWER) {
        LOG_INFO("ROS Command requested kill robot power... Issuing latched Emergency FET Shutdown");
        core1_kill_robot_power();
    }
}

static void swkill_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__KillSwitchReport *msg = (const riptide_msgs2__msg__KillSwitchReport *) msgin;

    if (msg->kill_switch_id == riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_RQT_CONTROLLER) {
        // Rviz is currently running
        has_rviz = true;
    }
}

static void leak_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;

    if (msg->data && !has_rviz) {
        // Detected leak and Rviz isn't present to respond to it... shutdown immediately
        LOG_INFO("Leak detected but no operator found to respond... Issuing latched Emergency FET Shutdown");
        core1_kill_robot_power();
    }
    else if (!msg->data) {
        // Force rechecking of rviz connection
        // Don't do this if there was a leak since the popup in rviz blocks sw kill pub
        has_rviz = false;
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

rcl_ret_t ros_update_battery_status(bq_mfg_info_t bq_pack_info) {
    riptide_msgs2__msg__BatteryStatus status;

    // push in the common cell info
    status.cell_name.data = "";
    status.cell_name.size = 0;
    status.serial = bq_pack_info.serial;

    // test for port and stbd

    bool side_det_is_high;
    if (core1_get_side_detect(&side_det_is_high)) {
        status.detect = (side_det_is_high ? riptide_msgs2__msg__BatteryStatus__DETECT_PORT :
                                            riptide_msgs2__msg__BatteryStatus__DETECT_STBD);
    }
    else {
        status.detect = riptide_msgs2__msg__BatteryStatus__DETECT_NONE;
    }

    // read cell info
    status.pack_voltage = ((float) core1_voltage()) / 1000.0;
    status.pack_current = ((float) core1_current()) / 1000.0;
    status.average_current = ((float) core1_avg_current()) / 1000.0;
    bool dsg;
    status.time_to_dischg = core1_time_remaining(&dsg);
    status.soc = core1_soc();

    // send out the ros message
    rcl_ret_t ret = rcl_publish(&battery_status_publisher, &status, NULL);

    RCSOFTRETCHECK(ret);

    return RCL_RET_OK;
}

rcl_ret_t ros_update_temp_humidity_publisher() {
    if (sht41_is_valid()) {
        std_msgs__msg__Float32 sht41_data;
        sht41_data.data = sht41_read_temp();
        RCSOFTRETCHECK(rcl_publish(&temp_status_publisher, &sht41_data, NULL));
        sht41_data.data = sht41_read_rh();
        RCSOFTRETCHECK(rcl_publish(&humidity_status_publisher, &sht41_data, NULL));
    }

    return RCL_RET_OK;
}

// ========================================
// ROS Core
// ========================================

char node_name[sizeof(PICO_TARGET_NAME) + 6];

rcl_ret_t ros_init(uint8_t board_id) {
    // ROS Core Initialization
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    snprintf(node_name, sizeof(node_name), PICO_TARGET_NAME "_%d", board_id);
    RCRETCHECK(rclc_node_init_default(&node, node_name, ROBOT_NAMESPACE, &support));

    // Node Initialization
    RCRETCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&firmware_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareStatus),
                                           FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&battery_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, BatteryStatus),
                                           BATTERY_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&temp_status_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                               TEMP_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&humidity_status_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                               HUMIDITY_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(
        &killswtich_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_SUBCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&electrical_command_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalCommand),
                                              ELECTRICAL_COMMAND_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&swkill_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, KillSwitchReport),
                                              SWKILL_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&leak_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                              LEAK_SUBSCRIBER_NAME));

    // Executor Initialization
    const int executor_num_handles = 4;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg,
                                              &killswitch_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &electrical_command_subscriber, &electrical_command_msg,
                                              &electrical_command_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &swkill_subscriber, &swkill_msg, &swkill_subscription_callback,
                                              ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &leak_subscriber, &leak_msg, &leak_subscription_callback,
                                              ON_NEW_DATA));

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
    RCSOFTCHECK(rcl_subscription_fini(&killswtich_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&electrical_command_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&battery_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&temp_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&humidity_status_publisher, &node));
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
