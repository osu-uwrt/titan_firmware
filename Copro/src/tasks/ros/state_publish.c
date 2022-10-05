#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include <riptide_msgs2/msg/actuator_status.h>
#include <riptide_msgs2/msg/electrical_readings.h>
#include <riptide_msgs2/msg/firmware_state.h>
#include <riptide_msgs2/msg/kill_switch_report.h>
#include <riptide_msgs2/msg/robot_state.h>
#include <std_msgs/msg/empty.h>

#include "basic_logger/logging.h"
#include "build_version.h"

#include "drivers/memmonitor.h"
#include "drivers/safety.h"
#include "hw/actuator.h"
#include "hw/depth_sensor.h"
#include "hw/dio.h"
#include "hw/esc_adc.h"
#include "tasks/cooling.h"
#include "hw/bmp280_temp.h"

#include "ros_private.h"


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

			firmware_state_msg.copro_faults = *fault_list_reg;
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

void state_publish_init(rclc_support_t *support, rcl_node_t *node, rclc_executor_t *executor) {
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

void state_publish_fini(rcl_node_t *node) {
	RCCHECK(rcl_publisher_fini(&electrical_readings_publisher, node));
	RCCHECK(rcl_publisher_fini(&robot_state_publisher, node));
	RCCHECK(rcl_publisher_fini(&firmware_state_publisher, node));
	RCCHECK(rcl_publisher_fini(&actuator_status_publisher, node));
	RCCHECK(rcl_timer_fini(&state_publish_timer));
}
