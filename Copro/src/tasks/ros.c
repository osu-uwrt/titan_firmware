#include "pico/stdlib.h"
#include <hardware/watchdog.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <riptide_msgs2/msg/actuator_status.h>
#include <riptide_msgs2/msg/actuator_command.h>
#include <riptide_msgs2/msg/depth.h>
#include <riptide_msgs2/msg/electrical_command.h>
#include <riptide_msgs2/msg/electrical_readings.h>
#include <riptide_msgs2/msg/firmware_state.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <riptide_msgs2/msg/lighting_command.h>
#include <riptide_msgs2/msg/pwm_stamped.h>
#include <riptide_msgs2/msg/robot_state.h>
#include <std_msgs/msg/empty.h>

#include "basic_logger/logging.h"
#include "build_version.h"
#include "pico_uart_transports.h"

#include "drivers/memmonitor.h"
#include "drivers/safety.h"
#include "hw/actuator.h"
#include "hw/balancer_adc.h"
#include "hw/depth_sensor.h"
#include "hw/dio.h"
#include "hw/esc_adc.h"
#include "hw/esc_pwm.h"
#include "tasks/ros.h"
#include "tasks/cooling.h"
#include "hw/bmp280_temp.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_FATAL("Failed status on in : %d. Aborting.",__LINE__,(int)temp_rc); panic("Unrecoverable ROS Error");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_SOFT_FAIL);}}

static void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
	ts->tv_sec = time_nanos / 1000000000;
	ts->tv_nsec = time_nanos % 1000000000;
}

// ========================================
// Depth Reading Callbacks
// ========================================

static rcl_publisher_t depth_publisher;
static riptide_msgs2__msg__Depth depth_msg;
static rcl_timer_t depth_publisher_timer;
static char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
static const float depth_variance = 0.003;  					// TODO: Load from config file or something
static const int depth_publish_rate_ms = 50;

static void depth_publisher_timer_callback(rcl_timer_t * timer, __unused int64_t last_call_time) {
	if (timer != NULL && depth_reading_valid()) {

		struct timespec ts;
		nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
		depth_msg.header.stamp.sec = ts.tv_sec;
		depth_msg.header.stamp.nanosec = ts.tv_nsec;

		depth_msg.depth = -depth_read();
		RCSOFTCHECK(rcl_publish(&depth_publisher, &depth_msg, NULL));
	}
}

static void depth_publisher_init(rclc_support_t *support, rcl_node_t *node, rclc_executor_t *executor) {
	RCCHECK(rclc_publisher_init(
		&depth_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth),
		"depth/raw",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_timer_init_default(
		&depth_publisher_timer,
		support,
		RCL_MS_TO_NS(depth_publish_rate_ms),
		depth_publisher_timer_callback));

	RCCHECK(rclc_executor_add_timer(executor, &depth_publisher_timer));

	depth_msg.header.frame_id.data = depth_frame;
	depth_msg.header.frame_id.capacity = sizeof(depth_frame);
	depth_msg.header.frame_id.size = strlen(depth_frame);
	depth_msg.variance = depth_variance;
}

static void depth_publisher_cleanup(rcl_node_t *node) {
	RCCHECK(rcl_publisher_fini(&depth_publisher, node));
	RCCHECK(rcl_timer_fini(&depth_publisher_timer));
}

// ========================================
// Sensor Reading Callback
// ========================================

static rcl_publisher_t heartbeat_publisher;
static std_msgs__msg__Empty heartbeat_msg;
static rcl_publisher_t electrical_readings_publisher;
static riptide_msgs2__msg__ElectricalReadings electrical_readings_msg;
static rcl_publisher_t robot_state_publisher;
static riptide_msgs2__msg__RobotState robot_state_msg;
static rcl_publisher_t firmware_state_publisher;
static riptide_msgs2__msg__FirmwareState firmware_state_msg;
static rcl_publisher_t actuator_status_publisher;
static riptide_msgs2__msg__ActuatorStatus actuator_status_msg;

static rcl_timer_t state_publish_timer;
static const int state_publish_rate_ms = 500;
static char copro_frame[] = ROBOT_NAMESPACE "/coprocessor";

static uint8_t actuator_to_ros_dropper_state(enum dropper_state dropper_state) {
	if (dropper_state == DROPPER_STATE_READY) {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_READY;
	} else if (dropper_state == DROPPER_STATE_DROPPING) {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPING;
	} else if (dropper_state == DROPPER_STATE_DROPPED) {
		return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPED;
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
	} else if (torpedo_state == TORPEDO_STATE_FIRED) {
		return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRED;
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

static void state_publish_callback(rcl_timer_t * timer, __unused int64_t last_call_time) {
	if (timer != NULL) {
		struct timespec ts;
		nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);

		{
			electrical_readings_msg.header.stamp.sec = ts.tv_sec;
			electrical_readings_msg.header.stamp.nanosec = ts.tv_nsec;

			if (esc_adc_initialized && !esc_adc_readng_stale) {
				for (int i = 0; i < 8; i++) {
					electrical_readings_msg.esc_current[i] = esc_adc_get_thruster_current(i);
				}
			} else {
				for (int i = 0; i < 8; i++) {
					electrical_readings_msg.esc_current[i] = riptide_msgs2__msg__ElectricalReadings__NO_READING;
				}
			}

			/*if (balancer_adc_initialized && !balancer_adc_readng_stale) {
				electrical_readings_msg.port_voltage = balancer_adc_get_port_voltage();
				electrical_readings_msg.stbd_voltage = balancer_adc_get_stbd_voltage();
				electrical_readings_msg.port_current = balancer_adc_get_port_current();
				electrical_readings_msg.stbd_current = balancer_adc_get_stbd_current();
				electrical_readings_msg.balanced_voltage = balancer_adc_get_balanced_voltage();
			} else {
				electrical_readings_msg.port_voltage = riptide_msgs2__msg__ElectricalReadings__NO_READING;
				electrical_readings_msg.stbd_voltage = riptide_msgs2__msg__ElectricalReadings__NO_READING;
				electrical_readings_msg.port_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;
				electrical_readings_msg.stbd_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;
				electrical_readings_msg.balanced_voltage = riptide_msgs2__msg__ElectricalReadings__NO_READING;
			}*/

			double reading = dio_get_battery_voltage_hack();
			electrical_readings_msg.port_voltage = reading;
			electrical_readings_msg.stbd_voltage = reading;
			electrical_readings_msg.balanced_voltage = reading;


			electrical_readings_msg.port_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;
			electrical_readings_msg.stbd_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;

			electrical_readings_msg.five_volt_voltage = riptide_msgs2__msg__ElectricalReadings__NO_READING;
			electrical_readings_msg.five_volt_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;
			electrical_readings_msg.twelve_volt_voltage = riptide_msgs2__msg__ElectricalReadings__NO_READING;
			electrical_readings_msg.twelve_volt_current = riptide_msgs2__msg__ElectricalReadings__NO_READING;

			RCSOFTCHECK(rcl_publish(&electrical_readings_publisher, &electrical_readings_msg, NULL));
		}

		{
			robot_state_msg.header.stamp.sec = ts.tv_sec;
			robot_state_msg.header.stamp.nanosec = ts.tv_nsec;

			robot_state_msg.kill_switch_inserted = !safety_kill_get_asserting_kill();
			robot_state_msg.aux_switch_inserted = dio_get_aux_switch();
			robot_state_msg.peltier_active = cooling_get_active();

			double temp;
			if (bmp280_temp_read(&temp)) {
				robot_state_msg.robot_temperature = temp;
			} else {
				robot_state_msg.robot_temperature = riptide_msgs2__msg__RobotState__NO_READING;
			}

			if (depth_reading_valid()) {
				robot_state_msg.water_temperature = depth_get_temperature();
			} else {
				robot_state_msg.water_temperature = riptide_msgs2__msg__RobotState__NO_READING;
			}

			RCSOFTCHECK(rcl_publish(&robot_state_publisher, &robot_state_msg, NULL));
		}

		{
			firmware_state_msg.header.stamp.sec = ts.tv_sec;
			firmware_state_msg.header.stamp.nanosec = ts.tv_nsec;

			firmware_state_msg.actuator_connected = actuator_is_connected();
			if (actuator_is_connected()){
				firmware_state_msg.actuator_faults = actuator_last_status.firmware_status.fault_list;
			} else {
				firmware_state_msg.actuator_faults = 0;
			}

			firmware_state_msg.copro_faults = *fault_list;
			firmware_state_msg.copro_memory_usage = memmonitor_get_total_use_percentage();
			firmware_state_msg.depth_sensor_initialized = depth_initialized;
			firmware_state_msg.peltier_cooling_threshold = cooling_threshold;

			firmware_state_msg.version_major = MAJOR_VERSION;
			firmware_state_msg.version_minor = MINOR_VERSION;

			firmware_state_msg.kill_switches_enabled = 0;
			firmware_state_msg.kill_switches_asserting_kill = 0;
			firmware_state_msg.kill_switches_needs_update = 0;
			firmware_state_msg.kill_switches_timed_out = 0;
			absolute_time_t now = get_absolute_time();
			for (int i = 0; i < riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES; i++) {
				if (kill_switch_states[i].enabled) {
					firmware_state_msg.kill_switches_enabled |= (1<<i);
				}

				if (kill_switch_states[i].asserting_kill) {
					firmware_state_msg.kill_switches_asserting_kill |= (1<<i);
				}
				
				if (kill_switch_states[i].needs_update) {
					firmware_state_msg.kill_switches_needs_update |= (1<<i);
				}

				if (kill_switch_states[i].needs_update && absolute_time_diff_us(now, kill_switch_states[i].update_timeout) < 0) {
					firmware_state_msg.kill_switches_timed_out |= (1<<i);
				}
			}

			RCSOFTCHECK(rcl_publish(&firmware_state_publisher, &firmware_state_msg, NULL));
		}

		{
			if (actuator_is_connected()) {
				actuator_status_msg.claw_state = actuator_to_ros_claw_state(actuator_last_status.claw_state);
				actuator_status_msg.torpedo1_state = actuator_to_ros_torpedo_state(actuator_last_status.torpedo1_state);
				actuator_status_msg.torpedo2_state = actuator_to_ros_torpedo_state(actuator_last_status.torpedo2_state);
				actuator_status_msg.dropper1_state = actuator_to_ros_dropper_state(actuator_last_status.dropper1_state);
				actuator_status_msg.dropper2_state = actuator_to_ros_dropper_state(actuator_last_status.dropper2_state);
			} else {
				actuator_status_msg.claw_state = riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
				actuator_status_msg.torpedo1_state = riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
				actuator_status_msg.torpedo2_state = riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
				actuator_status_msg.dropper1_state = riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
				actuator_status_msg.dropper2_state = riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
			}
			RCSOFTCHECK(rcl_publish(&actuator_status_publisher, &actuator_status_msg, NULL));
		}

		// Publishing a heartbeat with a reliable QOS to block if no connection forcing watchdog reset
		// This must succeed. Make it crash on failure and let the watchdog handle it
		RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
	}
}

static void state_publish_init(rclc_support_t *support, rcl_node_t *node, rclc_executor_t *executor) {
	RCCHECK(rclc_publisher_init(
		&heartbeat_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
		"copro_heartbeat",
		&rmw_qos_profile_default));

	RCCHECK(rclc_publisher_init(
		&electrical_readings_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ElectricalReadings),
		"state/electrical",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_publisher_init(
		&robot_state_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, RobotState),
		"state/robot",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_publisher_init(
		&firmware_state_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, FirmwareState),
		"state/firmware",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_publisher_init(
		&actuator_status_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus),
		"state/actuator",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_timer_init_default(
		&state_publish_timer,
		support,
		RCL_MS_TO_NS(state_publish_rate_ms),
		state_publish_callback));

	RCCHECK(rclc_executor_add_timer(executor, &state_publish_timer));

	electrical_readings_msg.header.frame_id.data = copro_frame;
	electrical_readings_msg.header.frame_id.capacity = sizeof(copro_frame);
	electrical_readings_msg.header.frame_id.size = strlen(copro_frame);
	
	robot_state_msg.header.frame_id.data = copro_frame;
	robot_state_msg.header.frame_id.capacity = sizeof(copro_frame);
	robot_state_msg.header.frame_id.size = strlen(copro_frame);

	firmware_state_msg.header.frame_id.data = copro_frame;
	firmware_state_msg.header.frame_id.capacity = sizeof(copro_frame);
	firmware_state_msg.header.frame_id.size = strlen(copro_frame);
}

static void state_publish_cleanup(rcl_node_t *node) {
	RCCHECK(rcl_publisher_fini(&electrical_readings_publisher, node));
	RCCHECK(rcl_publisher_fini(&robot_state_publisher, node));
	RCCHECK(rcl_publisher_fini(&firmware_state_publisher, node));
	RCCHECK(rcl_publisher_fini(&actuator_status_publisher, node));
	RCCHECK(rcl_timer_fini(&state_publish_timer));
}

// ========================================
// Subscriber Callbacks
// ========================================

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
static char software_kill_frame_str[SOFTWARE_KILL_FRAME_STR_SIZE] = {0};
static void software_kill_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__KillSwitchReport * msg = (const riptide_msgs2__msg__KillSwitchReport *)msgin;
	
	if (!rmw_uros_epoch_synchronized()){
        LOG_ERROR("Safety Kill Switch: No Time Synchronization for Comand Verification!");
        safety_raise_fault(FAULT_ROS_SOFT_FAIL);
        return;
    }

    // Make sure kill switch id is valid
    if (msg->kill_switch_id >= riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES || 
            msg->kill_switch_id == riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL) {
        LOG_WARN("Invalid kill switch id used %d", msg->kill_switch_id);
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    // Make sure frame id isn't too large
    if (msg->sender_id.size >= SOFTWARE_KILL_FRAME_STR_SIZE) {
        LOG_WARN("Software Kill Frame ID too large");
        safety_raise_fault(FAULT_ROS_BAD_COMMAND);
        return;
    }

    struct kill_switch_state* kill_entry = &kill_switch_states[msg->kill_switch_id];

    if (kill_entry->enabled && kill_entry->asserting_kill && !msg->switch_asserting_kill && 
            strncmp(kill_entry->locking_frame, msg->sender_id.data, SOFTWARE_KILL_FRAME_STR_SIZE)) {
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

static void subscriptions_init(rcl_node_t *node, rclc_executor_t *executor) {
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

	RCCHECK(rclc_subscription_init_best_effort(
		&software_kill_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, KillSwitchReport),
		"control/software_kill"));

	RCCHECK(rclc_executor_add_subscription(executor, &pwm_subscriber, &pwm_msg, &pwm_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &actuator_subscriber, &actuator_msg, &actuator_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &lighting_subscriber, &lighting_msg, &lighting_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &electrical_control_subscriber, &electrical_control_msg, &electrical_control_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(executor, &software_kill_subscriber, &software_kill_msg, &software_kill_subscription_callback, ON_NEW_DATA));

	software_kill_msg.sender_id.data = software_kill_frame_str;
	software_kill_msg.sender_id.capacity = SOFTWARE_KILL_FRAME_STR_SIZE;
	software_kill_msg.sender_id.size = 0;
}

static void subscriptions_fini(rcl_node_t *node){
	RCCHECK(rcl_subscription_fini(&pwm_subscriber, node));
	RCCHECK(rcl_subscription_fini(&actuator_subscriber, node));
	RCCHECK(rcl_subscription_fini(&lighting_subscriber, node));
	RCCHECK(rcl_subscription_fini(&electrical_control_subscriber, node));
	RCCHECK(rcl_subscription_fini(&software_kill_subscriber, node));
}

// ========================================
// Parameter Server
// ========================================

const rclc_parameter_options_t param_server_options = {
      .notify_changed_over_dds = true,
      .max_params = 13 };

static rclc_parameter_server_t param_server;

static void on_parameter_changed(Parameter * param)
{
	if (actuator_handle_parameter_change(param)) {
		// Nothing to be done on successful parameter change
	} else {
		LOG_WARN("Unexpected parameter %s with type %d changed", param->name.data, param->value.type);
		safety_raise_fault(FAULT_ROS_SOFT_FAIL);
	}
}

static void parameter_server_init(rcl_node_t *node, rclc_executor_t *executor) {
  	RCCHECK(rclc_parameter_server_init_with_option(&param_server, node, &param_server_options));
	RCCHECK(rclc_executor_add_parameter_server(executor, &param_server, on_parameter_changed));
	
	RCCHECK(actuator_create_parameters(&param_server));
	// TODO: Add cooling threshold as a parameter
}

static void parameter_server_fini(rcl_node_t *node){
	rclc_parameter_server_fini(&param_server, node);
}

// ========================================
// Public Methods
// ========================================

void ros_wait_for_connection(void) {
	// Make sure this is less than the watchdog timeout
    const int timeout_ms = 1000; 

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);
}

static rcl_allocator_t allocator = {0};
static rclc_support_t support = {0};
static rcl_node_t node = {0};
static rclc_executor_t executor = {0};

void ros_start(const char* namespace) {
    allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "coprocessor_node", namespace, &support));

	// Before starting anything else, synchronize time
	RCCHECK(rmw_uros_sync_session(2000));

	// create executor
	const uint num_executor_tasks = 7 + RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, num_executor_tasks, &allocator));

	parameter_server_init(&node, &executor);
	depth_publisher_init(&support, &node, &executor);
	state_publish_init(&support, &node, &executor);
	subscriptions_init(&node, &executor);
}

void ros_spin_ms(long ms) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms)));
}

void ros_cleanup(void) {
	parameter_server_fini(&node);
	depth_publisher_cleanup(&node);
	state_publish_cleanup(&node);
	subscriptions_fini(&node);

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
}