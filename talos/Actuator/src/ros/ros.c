#include "pico/stdlib.h"

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <riptide_msgs2/msg/firmware_status.h>
#include <riptide_msgs2/msg/actuator_status.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

#include "build_version.h"
#include "basic_logger/logging.h"

#include "ros/ros.h"
#include "actuators/dropper.h"
#include "actuators/torpedo.h"
#include "actuators/claw.h"

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
rcl_publisher_t actuator_status_publisher;
riptide_msgs2__msg__ActuatorStatus actuator_status_msg;
// TODO: Add node specific items here

// ========================================
// Executor Callbacks
// ========================================

static void killswitch_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    safety_kill_switch_update(ROS_KILL_SWITCH, msg->data, true);
}

// TODO: Add in node specific tasks here

// ========================================
// Public Task Methods (called in main tick)
// ========================================

static uint8_t actuator_to_ros_dropper_state(enum dropper_state dropper_state) {
	if (dropper_state == DROPPER_STATE_READY) {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_READY;
	} else if (dropper_state == DROPPER_STATE_DROPPING) {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPING;
	} else {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
	}
}

static uint8_t actuator_to_ros_torpedo_state(enum torpedo_state torpedo_state) {
	if (torpedo_state == TORPEDO_STATE_DISARMED) {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_DISARMED;
	} else if (torpedo_state == TORPEDO_STATE_CHARGING) {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_CHARGING;
	} else if (torpedo_state == TORPEDO_STATE_READY) {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_CHARGED;
	} else if (torpedo_state == TORPEDO_STATE_FIRING) {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRING;
	} else {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
	}
}

static uint8_t actuator_to_ros_claw_state(enum claw_state claw_state) {
	if (claw_state == CLAW_STATE_UNKNOWN_POSITION) {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_UNKNOWN;
	} else if (claw_state == CLAW_STATE_OPENED) {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_OPENED;
	} else if (claw_state == CLAW_STATE_CLOSED) {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSED;
	} else if (claw_state == CLAW_STATE_OPENING) {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_OPENING;
	} else if (claw_state == CLAW_STATE_CLOSING) {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_CLOSING;
	} else {
		return riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
	}
}

rcl_ret_t ros_update_actuator_status(uint8_t client_id) {
	actuator_status_msg.claw_state = actuator_to_ros_claw_state(claw_get_state());
	actuator_status_msg.torpedo1_state = actuator_to_ros_torpedo_state(torpedo_get_state(1));
	actuator_status_msg.torpedo2_state = actuator_to_ros_torpedo_state(torpedo_get_state(2));
	actuator_status_msg.dropper1_state = actuator_to_ros_dropper_state(dropper_get_state(1));
	actuator_status_msg.dropper2_state = actuator_to_ros_dropper_state(dropper_get_state(2));

	RCRETCHECK(rcl_publish(&actuator_status_publisher, &actuator_status_msg, NULL));

    return RCL_RET_OK;
}

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

// TODO: Add in node specific tasks here

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
		&actuator_status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus),
		"state/actuator",
		&rmw_qos_profile_sensor_data));

    // Executor Initialization
    const int executor_num_handles = 5;
    RCRETCHECK(rclc_executor_init(&executor, &support.context, executor_num_handles, &allocator));
    RCRETCHECK(rclc_executor_add_subscription(&executor, &killswtich_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));

    // TODO: Modify this method with node specific objects
    //RCRETCHECK(ros_claw_init(&executor, &node, &support));
    //RCRETCHECK(ros_torpedo_dropper_init(&executor, &node, &support));

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

    //ros_claw_fini(&node);
    //ros_torpedo_dropper_fini(&node);

    RCSOFTCHECK(rcl_publisher_fini(&actuator_status_publisher, &node));
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

