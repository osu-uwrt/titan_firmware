#include "ros.h"

#include "analog_io.h"

#include "driver/mcp3426.h"
#include "driver/sht41.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <riptide_msgs2/msg/electrical_readings.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>

#include <math.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

// ========================================
// Global Definitions
// ========================================

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_PUBLISHER_NAME "state/kill"
#define SOFT_KILL_SUBSCRIBER_NAME "command/software_kill"
#define ELECTRICAL_READING_NAME "state/electrical"
#define PHYSICAL_KILL_NOTIFY_PUBLISHER_NAME "state/physkill_notify"
#define ELECTRICAL_COMMAND_SUBSCRIBER_NAME "command/electrical"
#define TEMP_STATUS_PUBLISHER_NAME "state/temp/powerboard"
#define HUMIDITY_STATUS_PUBLISHER_NAME "state/humidity/powerboard"

#define AUX_SWITCH_PUBLISHER_NAME "state/aux"

#define BALANCING_FEEDBACK_PUBLISHER_NAME "state/batteries_balanced"

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
rcl_publisher_t electrical_reading_publisher;
riptide_msgs2__msg__ElectricalReadings electrical_reading_msg = { 0 };
rcl_subscription_t elec_command_subscriber;
riptide_msgs2__msg__ElectricalCommand elec_command_msg;
rcl_publisher_t aux_switch_publisher;
rcl_publisher_t balancing_feedback_publisher;
rcl_publisher_t temp_status_publisher;
rcl_publisher_t humidity_status_publisher;

// Kill switch
rcl_publisher_t killswitch_publisher;
rcl_publisher_t physkill_notify_publisher;
static bool last_physical_kill_asserting_kill = true;
rcl_subscription_t software_kill_subscriber;
riptide_msgs2__msg__KillSwitchReport software_kill_msg;
char software_kill_frame_str[SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE + 1] = { 0 };

// ========================================
// Executor Callbacks
// ========================================

static void software_kill_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__KillSwitchReport *msg = (const riptide_msgs2__msg__KillSwitchReport *) msgin;

    // Make sure kill switch id is valid
    if (msg->kill_switch_id >= riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES ||
        msg->kill_switch_id == riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL) {
        LOG_WARN("Invalid kill switch id used %d", msg->kill_switch_id);
        safety_raise_fault_with_arg(FAULT_ROS_BAD_COMMAND, msg->kill_switch_id);
        return;
    }

    // Make sure frame id isn't too large
    if (msg->sender_id.size >= SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE) {
        LOG_WARN("Software Kill Frame ID too large");
        safety_raise_fault_with_arg(FAULT_ROS_BAD_COMMAND, msg->sender_id.size);
        return;
    }

    struct kill_switch_state *kill_entry = &kill_switch_states[msg->kill_switch_id];

    if (kill_entry->enabled && kill_entry->asserting_kill && !msg->switch_asserting_kill &&
        strncmp(kill_entry->locking_frame, msg->sender_id.data, SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE)) {
        LOG_WARN("Invalid frame ID to unlock kill switch %d ('%s' expected, '%s' requested)", msg->kill_switch_id,
                 kill_entry->locking_frame, msg->sender_id.data);
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

static void elec_command_subscription_callback(const void *msgin) {
    const riptide_msgs2__msg__ElectricalCommand *msg = (const riptide_msgs2__msg__ElectricalCommand *) msgin;
    if (msg->command == riptide_msgs2__msg__ElectricalCommand__ENABLE_FANS) {
        gpio_put(FAN_SWITCH_PIN, true);
    }
    else if (msg->command == riptide_msgs2__msg__ElectricalCommand__DISABLE_FANS) {
        gpio_put(FAN_SWITCH_PIN, false);
    }
}

// ========================================
// Public Task Methods (called in main tick)
// ========================================

static float calc_actual_voltage(float adc_voltage, bool is_3v3) {
    float top = 10000.0;
    float bottom;

    if (is_3v3) {
        bottom = 10000.0;
    }
    else {
        bottom = 1000.0;
    }
    return adc_voltage * 1.0 / (bottom / (bottom + top));
}

rcl_ret_t ros_publish_electrical_readings() {
    bool stbd_pwring = gpio_get(STBD_STAT_PIN);
    bool port_pwring = gpio_get(PORT_STAT_PIN);
    electrical_reading_msg.port_voltage = analog_io_read_port_meas();
    electrical_reading_msg.stbd_voltage = analog_io_read_stbd_meas();
    electrical_reading_msg.three_volt_voltage = calc_actual_voltage(mcp3426_read_voltage(MCP3426_CHANNEL_1), true);
    electrical_reading_msg.balanced_voltage = calc_actual_voltage(mcp3426_read_voltage(MCP3426_CHANNEL_2), false);
    electrical_reading_msg.twelve_volt_voltage = calc_actual_voltage(mcp3426_read_voltage(MCP3426_CHANNEL_3), false);
    electrical_reading_msg.five_volt_voltage = calc_actual_voltage(mcp3426_read_voltage(MCP3426_CHANNEL_4), false);
    electrical_reading_msg.port_powering = port_pwring;
    electrical_reading_msg.stbd_powering = stbd_pwring;

    RCSOFTRETCHECK(rcl_publish(&electrical_reading_publisher, &electrical_reading_msg, NULL));

    std_msgs__msg__Bool balancing_feedback_msg;
    if (port_pwring && stbd_pwring) {
        balancing_feedback_msg.data = true;
    }
    else {
        balancing_feedback_msg.data = false;
    }
    RCSOFTRETCHECK(rcl_publish(&balancing_feedback_publisher, &balancing_feedback_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_publish_auxswitch() {
    std_msgs__msg__Bool auxswitch_msg;
    auxswitch_msg.data = gpio_get(AUX_SWITCH_PIN);
    RCSOFTRETCHECK(rcl_publish(&aux_switch_publisher, &auxswitch_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_publish_killswitch() {
    std_msgs__msg__Bool killswitch_msg = { .data = safety_kill_get_asserting_kill() };

    RCSOFTRETCHECK(rcl_publish(&killswitch_publisher, &killswitch_msg, NULL));

    // Notify the physical kill state
    // This is primarily used to report to the actuator/camera cage boards to flash the LEDs on kill switch
    // insertion/removal
    bool value = safety_interface_physical_kill_asserting_kill;
    if (last_physical_kill_asserting_kill != value) {
        last_physical_kill_asserting_kill = value;
        std_msgs__msg__Bool physkill_notify_msg = { .data = value };
        RCSOFTRETCHECK(rcl_publish(&physkill_notify_publisher, &physkill_notify_msg, NULL));
    }

    return RCL_RET_OK;
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

    for (int i = 0; i < riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES; i++) {
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

rcl_ret_t ros_update_temp_humidity_publisher() {
    if (sht41_is_valid()) {
        std_msgs__msg__Float32 sht41_msg;
        sht41_msg.data = sht41_read_temp();
        RCSOFTRETCHECK(rcl_publish(&temp_status_publisher, &sht41_msg, NULL));
        sht41_msg.data = sht41_read_rh();
        RCSOFTRETCHECK(rcl_publish(&humidity_status_publisher, &sht41_msg, NULL));
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
    RCRETCHECK(rclc_publisher_init_default(&heartbeat_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), HEARTBEAT_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&firmware_status_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareStatus),
                                           FIRMWARE_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &killswitch_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), KILLSWITCH_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&physkill_notify_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                           PHYSICAL_KILL_NOTIFY_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&electrical_reading_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalReadings),
                                           ELECTRICAL_READING_NAME));

    RCRETCHECK(rclc_publisher_init_default(
        &aux_switch_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), AUX_SWITCH_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_default(&balancing_feedback_publisher, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                           BALANCING_FEEDBACK_PUBLISHER_NAME));

    RCRETCHECK(rclc_subscription_init_best_effort(&software_kill_subscriber, &node,
                                                  ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, KillSwitchReport),
                                                  SOFT_KILL_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_subscription_init_default(&elec_command_subscriber, &node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalCommand),
                                              ELECTRICAL_COMMAND_SUBSCRIBER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&temp_status_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                               TEMP_STATUS_PUBLISHER_NAME));

    RCRETCHECK(rclc_publisher_init_best_effort(&humidity_status_publisher, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                               HUMIDITY_STATUS_PUBLISHER_NAME));
    // Executor Initialization
    const int executor_num_handles = 2;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &software_kill_subscriber, &software_kill_msg,
                                              &software_kill_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &elec_command_subscriber, &elec_command_msg,
                                              &elec_command_subscription_callback, ON_NEW_DATA));

    // Populate messages
    software_kill_msg.sender_id.data = software_kill_frame_str;
    software_kill_msg.sender_id.capacity = sizeof(software_kill_frame_str);
    software_kill_msg.sender_id.size = 0;

    // Make sure that if the physical kill switch is enabled, a notify is sent out
    last_physical_kill_asserting_kill = true;

    return RCL_RET_OK;
}

void ros_spin_executor(void) {
    rclc_executor_spin_some(&executor, 0);
}

void ros_fini(void) {
    RCSOFTCHECK(rcl_subscription_fini(&elec_command_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&software_kill_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&temp_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&humidity_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&electrical_reading_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&killswitch_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&physkill_notify_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&heartbeat_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&firmware_status_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&balancing_feedback_publisher, &node));
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
