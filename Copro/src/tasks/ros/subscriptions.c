#include "pico/stdlib.h"
#include <hardware/watchdog.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include <riptide_msgs2/msg/actuator_command.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <riptide_msgs2/msg/lighting_command.h>
#include <riptide_msgs2/msg/pwm_stamped.h>

#include "basic_logger/logging.h"

#include "drivers/safety.h"
#include "hw/actuator.h"
#include "hw/esc_pwm.h"
#include "hw/dio.h"
#include "tasks/cooling.h"

#include "ros_private.h"


static rcl_subscription_t pwm_subscriber;
static riptide_msgs2__msg__PwmStamped pwm_msg;
static void pwm_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__PwmStamped * msg = (const riptide_msgs2__msg__PwmStamped *)msgin;
	esc_pwm_update_thrusters(msg);
}

static rcl_subscription_t actuator_subscriber;
static riptide_msgs2__msg__ActuatorCommand actuator_msg;
static void actuator_subscription_callback(const void * msgin)
{
	__unused const riptide_msgs2__msg__ActuatorCommand * msg = (const riptide_msgs2__msg__ActuatorCommand *)msgin;

	if (msg->open_claw) {
		actuator_open_claw();
	}
	if (msg->close_claw) {
		actuator_close_claw();
	}
	if (msg->arm_torpedo) {
		actuator_arm_torpedo();
	}
	if (msg->disarm_torpedo) {
		actuator_disarm_torpedo();
	}
	if (msg->fire_torpedo_1) {
		actuator_fire_torpedo(1);
	}
	if (msg->fire_torpedo_2) {
		actuator_fire_torpedo(2);
	}
	if (msg->drop_1) {
		actuator_drop_marker(1);
	}
	if (msg->drop_2) {
		actuator_drop_marker(2);
	}
	if (msg->clear_dropper_status) {
		actuator_clear_dropper_status();
	}
	if (msg->reset_actuators) {
		actuator_reset_actuators();
	}
}

static rcl_subscription_t lighting_subscriber;
static riptide_msgs2__msg__LightingCommand lighting_msg;
static void lighting_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__LightingCommand * msg = (const riptide_msgs2__msg__LightingCommand *)msgin;
	LOG_WARN("Unimplemented Command: Change Lighting (1: %d, 2: %d)", msg->light_1_brightness_percentage, msg->light_2_brightness_percentage);
	// TODO: Implement in DIO
}

static rcl_subscription_t electrical_control_subscriber;
static riptide_msgs2__msg__ElectricalCommand electrical_control_msg;
static void electrical_control_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__ElectricalCommand * msg = (const riptide_msgs2__msg__ElectricalCommand *)msgin;

	if (msg->cooling_threshold != riptide_msgs2__msg__ElectricalCommand__NO_COOLING_THRESH) {
		cooling_threshold = msg->cooling_threshold;
		LOG_INFO("Setting cooling threshold: %d", cooling_threshold);
	}

	if (msg->reset_copro){
		safety_notify_software_reset();
		watchdog_reboot(0, 0, 0);
	}

	if (msg->reset_computer) {
		dio_toggle_mobo_power();
	}

	if (msg->power_cycle_robot) {
		LOG_WARN("Unimplemented command: Power Cycle Robot");
		// TODO: Implement in DIO
	}

	if (msg->kill_robot_electrical) {
		LOG_WARN("Unimplemented command: Kill Robot Electrical");
		// TODO: Implement in DIO
	}
}

static rcl_subscription_t software_kill_subscriber;
static riptide_msgs2__msg__KillSwitchReport software_kill_msg;
static char software_kill_frame_str[SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE] = {0};
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

void subscriptions_init(rcl_node_t *node, rclc_executor_t *executor) {
	RCCHECK(rclc_subscription_init_best_effort(
		&software_kill_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, KillSwitchReport),
		"control/software_kill"));

	RCCHECK(rclc_subscription_init_best_effort(
		&pwm_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, PwmStamped),
		"command/pwm"));

	RCCHECK(rclc_subscription_init_default(
		&actuator_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorCommand),
		"command/actuator"));

	RCCHECK(rclc_subscription_init_default(
		&lighting_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, LightingCommand),
		"command/lighting"));

	RCCHECK(rclc_subscription_init_default(
		&electrical_control_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalCommand),
		"control/electrical"));


	RCCHECK(rclc_executor_add_subscription(executor, &pwm_subscriber, &pwm_msg, &pwm_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &actuator_subscriber, &actuator_msg, &actuator_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &lighting_subscriber, &lighting_msg, &lighting_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &electrical_control_subscriber, &electrical_control_msg, &electrical_control_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &software_kill_subscriber, &software_kill_msg, &software_kill_subscription_callback, ON_NEW_DATA));

	software_kill_msg.sender_id.data = software_kill_frame_str;
	software_kill_msg.sender_id.capacity = SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE;
	software_kill_msg.sender_id.size = 0;
}

void subscriptions_fini(rcl_node_t *node) {
	RCCHECK(rcl_subscription_fini(&pwm_subscriber, node));
	RCCHECK(rcl_subscription_fini(&actuator_subscriber, node));
	RCCHECK(rcl_subscription_fini(&lighting_subscriber, node));
	RCCHECK(rcl_subscription_fini(&electrical_control_subscriber, node));
	RCCHECK(rcl_subscription_fini(&software_kill_subscriber, node));
}