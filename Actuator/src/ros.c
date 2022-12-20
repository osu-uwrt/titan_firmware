#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include "drivers/safety.h"
#include "actuators/torpedo.h"
#include "actuators/dropper.h"
#include "actuators/claw.h"
#include "actuators/arm_state.h"

#include "basic_logger/logging.h"
#include "pico_uart_transports.h"

#include <riptide_msgs2/action/actuate_droppers.h>
#include <riptide_msgs2/action/change_claw_state.h>
#include <riptide_msgs2/action/actuate_torpedos.h>
#include <riptide_msgs2/action/arm_torpedo_dropper.h>
#include <riptide_msgs2/msg/actuator_status.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_FATAL("Failed status on in " __FILE__ ":%d : %d. Aborting.",__LINE__,(int)temp_rc); panic("Unrecoverable ROS Error");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_SOFT_FAIL);}}

static rcl_allocator_t allocator = {0};
static rclc_support_t support = {0};
static rcl_node_t node = {0};
static rclc_executor_t executor = {0};
static rcl_timer_t goal_timer = {0};
static rcl_timer_t publisher_timer = {0};


//
// Parameter server
//

static const rclc_parameter_options_t param_server_options = {
      .notify_changed_over_dds = false,
      .max_params = 13 };

static rclc_parameter_server_t param_server = {0};


#define RC_RETURN_CHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return temp_rc;}}
rcl_ret_t actuator_create_parameters(rclc_parameter_server_t *param_server) {
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "claw_timing_ms", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "dropper_active_timing_ms", RCLC_PARAMETER_INT));

	RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo1_coil1_on_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo1_coil1_2_delay_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo1_coil2_on_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo1_coil2_3_delay_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo1_coil3_on_timing_us", RCLC_PARAMETER_INT));

    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo2_coil1_on_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo2_coil1_2_delay_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo2_coil2_on_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo2_coil2_3_delay_timing_us", RCLC_PARAMETER_INT));
    RC_RETURN_CHECK(rclc_add_parameter(param_server, "torpedo2_coil3_on_timing_us", RCLC_PARAMETER_INT));

    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "claw_timing_ms", 4500));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "dropper_active_timing_ms", 250));

    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo1_coil1_on_timing_us", 23000));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo1_coil1_2_delay_timing_us", 250));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo1_coil2_on_timing_us", 15000));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo1_coil2_3_delay_timing_us", 250));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo1_coil3_on_timing_us", 13000));

    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo2_coil1_on_timing_us", 23000));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo2_coil1_2_delay_timing_us", 250));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo2_coil2_on_timing_us", 15000));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo2_coil2_3_delay_timing_us", 250));
    RC_RETURN_CHECK(rclc_parameter_set_int(param_server, "torpedo2_coil3_on_timing_us", 13000));

    return RCL_RET_OK;
}


#define IS_VALID_TIMING(num) ((num) > 0 && (num) < (1<<16))

bool handle_parameter_change(const Parameter * param) {
	    // All parameters are int types
    if (param->value.type != RCLC_PARAMETER_INT) {
        return false;
    }

	if (!strcmp(param->name.data, "dropper_active_timing_ms")) {
        if (IS_VALID_TIMING(param->value.integer_value)) {
            return dropper_set_timings(param->value.integer_value);
		} else {
            return false;
        }
    } else if (!strcmp(param->name.data, "claw_timing_ms")) {
        if (IS_VALID_TIMING(param->value.integer_value)) {
			claw_set_timings(param->value.integer_value, param->value.integer_value);
            return true;
        } else {
            return false;
        }
    }
	#define ELSE_IF_TORPEDO_PARAMETER(torp_num, param_name, timing_type) \
        else if (!strcmp(param->name.data, "torpedo" #torp_num "_" #param_name "_timing_us")) { \
            if (IS_VALID_TIMING(param->value.integer_value)) { \
                torpedo_set_timings(torp_num, timing_type, param->value.integer_value); \
                return true; \
            } else { \
                return false; \
            } \
        }

	ELSE_IF_TORPEDO_PARAMETER(1, coil1_on,      ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME)
    ELSE_IF_TORPEDO_PARAMETER(1, coil1_2_delay, ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME)
    ELSE_IF_TORPEDO_PARAMETER(1, coil2_on,      ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME)
    ELSE_IF_TORPEDO_PARAMETER(1, coil2_3_delay, ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME)
    ELSE_IF_TORPEDO_PARAMETER(1, coil3_on,      ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME)
    ELSE_IF_TORPEDO_PARAMETER(2, coil1_on,      ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME)
    ELSE_IF_TORPEDO_PARAMETER(2, coil1_2_delay, ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME)
    ELSE_IF_TORPEDO_PARAMETER(2, coil2_on,      ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME)
    ELSE_IF_TORPEDO_PARAMETER(2, coil2_3_delay, ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME)
    ELSE_IF_TORPEDO_PARAMETER(2, coil3_on,      ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME)

	return false;
}

static bool on_parameter_changed(__unused const Parameter * old_param, const Parameter * new_param, __unused void * context)
{
	if (new_param == NULL) {
		return false;
	}

	if (handle_parameter_change(new_param)) {
		// Nothing to be done on successful parameter change
		return true;
	} else {
		LOG_WARN("Unexpected parameter %s with type %d changed", new_param->name.data, new_param->value.type);
		safety_raise_fault(FAULT_ROS_SOFT_FAIL);
		return true;
	}
}

void parameter_server_init(rcl_node_t *node, rclc_executor_t *executor) {
  	RCCHECK(rclc_parameter_server_init_with_option(&param_server, node, &param_server_options));
	RCCHECK(rclc_executor_add_parameter_server(executor, &param_server, on_parameter_changed));

	RCCHECK(actuator_create_parameters(&param_server));
	// TODO: Add cooling threshold as a parameter
}

void parameter_server_fini(rcl_node_t *node){
	rclc_parameter_server_fini(&param_server, node);
}

//
// Arm Dropper Torpedo
//

static rclc_action_server_t arm_torpedo_dropper_server;
static rclc_action_goal_handle_t * arm_torpedo_dropper_goal_handle = NULL;
static riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request arm_torpedo_dropper_ros_goal_request[10];

rcl_ret_t arm_torpedo_dropper_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;

 	riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request * req =
    	(riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *) goal_handle->ros_goal_request;

	if (arm_torpedo_dropper_goal_handle != NULL) {
		LOG_WARN("arm torpedo dropper goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	bool result;
	if(req->goal.arm_droppers)  {
		result = dropper_arm();
	} else if(req->goal.arm_torpedos) {
		result = torpedo_arm();
	} else {
		result = false;
		LOG_DEBUG("Neither arm dropper or arm toprpedo set to true. Rejecting arm torpedo/dropper goal.");
	}

	if(result)  {
		LOG_DEBUG("Arm dropper/torpedo worked. Accepted arm torpedo/dropper goal");
		arm_torpedo_dropper_goal_handle = goal_handle;
		return RCL_RET_ACTION_GOAL_ACCEPTED;
	} else {
		LOG_DEBUG("Arm dropper/torpedo failed. Rejecting arm torpedo/dropper goal");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}
}

bool arm_torpedo_dropper_cancel(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;
  (void) goal_handle;
	riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request * req =
    	(riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *) goal_handle->ros_goal_request;

	bool cancel;
	if(req->goal.arm_torpedos) {
		cancel = torpedo_disarm();
	} else if(req->goal.arm_droppers) {
		cancel = dropper_disarm();
	} else {
		cancel = false;
		LOG_WARN("Neither arm dropper or arm toprpedo set to true. Invalid arm torpedo/dropper goal.");
	}

	if(cancel) {
		LOG_DEBUG("cancelling arm torpedo/dropper goal.");
		arm_torpedo_dropper_goal_handle = NULL;
	}

	return cancel;
}

void arm_torpedo_dropper_init() {
    RCCHECK(
        rclc_action_server_init_default(
        &arm_torpedo_dropper_server,
        &node,
        &support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ArmTorpedoDropper),
        "arm_torpedo_dropper"
    ));

	RCCHECK(rclc_executor_add_action_server(
        &executor,
        &arm_torpedo_dropper_server,
        1,
        arm_torpedo_dropper_ros_goal_request,
        sizeof(riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request),
        arm_torpedo_dropper_goal_request,
        arm_torpedo_dropper_cancel,
        (void *) &arm_torpedo_dropper_server));
}

//
// Actuate Dropper Action Server
//

static rclc_action_server_t actuator_dropper_server;
static rclc_action_goal_handle_t * actuate_dropper_goal_handle = NULL;
static riptide_msgs2__action__ActuateDroppers_SendGoal_Request actuate_droppers_ros_goal_request[10];

rcl_ret_t actuate_dropper_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;

 	riptide_msgs2__action__ActuateDroppers_SendGoal_Request * req =
    	(riptide_msgs2__action__ActuateDroppers_SendGoal_Request *) goal_handle->ros_goal_request;

	if (actuate_dropper_goal_handle != NULL) {
		LOG_WARN("Actuate dropper goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	uint8_t dropper = req->goal.dropper_id;

	bool result = dropper_drop_marker(dropper);

	if(result)  {
		LOG_DEBUG("Dropper worked. Accepted actuate dropper goal");
		actuate_dropper_goal_handle = goal_handle;
		return RCL_RET_ACTION_GOAL_ACCEPTED;
	} else {
		LOG_DEBUG("Dropper failed. Rejecting actuate dropper goal");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}
}

bool actuate_dropper_cancel(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;
  (void) goal_handle;
	// Cancels aren't allowed
	return false;
}

void actutate_dropper_init() {
    RCCHECK(
        rclc_action_server_init_default(
        &actuator_dropper_server,
        &node,
        &support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ActuateDroppers),
        "actuate_dropper"
    ));

	RCCHECK(rclc_executor_add_action_server(
        &executor,
        &actuator_dropper_server,
        1,
        actuate_droppers_ros_goal_request,
        sizeof(riptide_msgs2__action__ActuateDroppers_SendGoal_Request),
        actuate_dropper_goal_request,
        actuate_dropper_cancel,
        (void *) &actuator_dropper_server));
}

//
// Actuate Torpedos
//

static rclc_action_server_t actuate_torpedo_server;
static rclc_action_goal_handle_t * actuate_torpedo_goal_handle = NULL;
static riptide_msgs2__action__ActuateTorpedos_SendGoal_Request actuate_torpedo_ros_goal_request[10];

rcl_ret_t actuate_torpedos_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;

 	riptide_msgs2__action__ActuateTorpedos_SendGoal_Request * req =
    	(riptide_msgs2__action__ActuateTorpedos_SendGoal_Request *) goal_handle->ros_goal_request;

	if (actuate_torpedo_goal_handle != NULL) {
		LOG_WARN("Actuate torpedo goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	uint8_t torpedo = req->goal.torpedo_id;

	 bool result = torpedo_fire(torpedo);

	if(result)  {
		LOG_DEBUG("Torpedo fired. Accepted torpedo goal");
		actuate_dropper_goal_handle = goal_handle;
		return RCL_RET_ACTION_GOAL_ACCEPTED;
	} else {
		LOG_DEBUG("Torpedo failed. Rejecting torpedo goal");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}
}

bool actuate_torpedos_cancel(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;
  (void) goal_handle;
	  (void) goal_handle;
	// Cancels aren't allowed
	return false;
}

void actuate_torpedos_init() {
    RCCHECK(
        rclc_action_server_init_default(
        &actuate_torpedo_server,
        &node,
        &support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ActuateTorpedos),
        "actuate_torpedo"
    ));

	RCCHECK(rclc_executor_add_action_server(
        &executor,
        &actuate_torpedo_server,
        1,
        actuate_torpedo_ros_goal_request,
        sizeof(riptide_msgs2__action__ActuateTorpedos_SendGoal_Request),
        actuate_torpedos_goal_request,
        actuate_torpedos_cancel,
        (void *) &actuate_torpedo_server));
}

//
// Change Claw State Functions
//

static rclc_action_server_t change_claw_state_server;
static rclc_action_goal_handle_t * change_claw_state_goal_handle = NULL;
static riptide_msgs2__action__ChangeClawState_SendGoal_Request change_claw_state_ros_goal_request[10];

rcl_ret_t change_claw_state_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;

 	riptide_msgs2__action__ChangeClawState_SendGoal_Request * req =
    	(riptide_msgs2__action__ChangeClawState_SendGoal_Request *) goal_handle->ros_goal_request;

	if (change_claw_state_goal_handle != NULL) {
		LOG_WARN("Change claw state goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	bool claw_open_param = req->goal.clawopen;

	bool result;
	if (claw_open_param) {
		result = claw_open();
	} else {
		result = claw_close();
	}

	if(result)  {
		LOG_DEBUG("Accepting change claw state goal");
		change_claw_state_goal_handle = goal_handle;
		return RCL_RET_ACTION_GOAL_ACCEPTED;
	} else {
		LOG_DEBUG("Rejecting change claw state goal!");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}
}

bool change_claw_state_cancel(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;
  (void) goal_handle;
	// Cancels aren't allowed
	return false;
}

void change_claw_state_init() {
    RCCHECK(
        rclc_action_server_init_default(
        &change_claw_state_server,
        &node,
        &support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ChangeClawState),
        "change_claw_state"
    ));

	RCCHECK(rclc_executor_add_action_server(
        &executor,
        &change_claw_state_server,
        1,
        change_claw_state_ros_goal_request,
        sizeof(riptide_msgs2__action__ChangeClawState_SendGoal_Request),
        change_claw_state_goal_request,
        change_claw_state_cancel,
        (void *) &change_claw_state_server));
}

// Actuator Publisher
static rcl_publisher_t actuator_status_publisher;
static riptide_msgs2__msg__ActuatorStatus actuator_status_msg;

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

void ros_update_publisher(__unused rcl_timer_t *timer, __unused int64_t last_call_time) {
	actuator_status_msg.claw_state = actuator_to_ros_claw_state(claw_get_state());
	actuator_status_msg.torpedo1_state = actuator_to_ros_torpedo_state(torpedo_get_state(1));
	actuator_status_msg.torpedo2_state = actuator_to_ros_torpedo_state(torpedo_get_state(2));
	actuator_status_msg.dropper1_state = actuator_to_ros_dropper_state(dropper_get_state(1));
	actuator_status_msg.dropper2_state = actuator_to_ros_dropper_state(dropper_get_state(2));

	RCSOFTCHECK(rcl_publish(&actuator_status_publisher, &actuator_status_msg, NULL));
}

//
// Generic ROS functions
//

void ros_wait_for_connection(void) {
	// Make sure this is less than the watchdog timeout
    const int timeout_ms = 1000;

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);
}

void ros_update_actions(__unused rcl_timer_t *timer, __unused int64_t last_call_time) {
	if(actuate_dropper_goal_handle != NULL) {
		riptide_msgs2__action__ActuateDroppers_SendGoal_Request * req =
    		(riptide_msgs2__action__ActuateDroppers_SendGoal_Request *) actuate_dropper_goal_handle->ros_goal_request;

		uint8_t dropper = req->goal.dropper_id;
		enum dropper_state dropper_state = dropper_get_state(dropper);

		LOG_DEBUG("running actuate dropper goal");
		if(dropper_state == DROPPER_STATE_READY) {
			LOG_DEBUG("dropper dropped");
			rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
        	riptide_msgs2__action__ActuateDroppers_GetResult_Response response = {0};

			rcl_ret_t rc = rclc_action_send_result(actuate_dropper_goal_handle, goal_state, &response);

			switch(rc) {
				case RCL_RET_OK:
					LOG_DEBUG("ActuateDropper goal finished and result accepted.");
					actuate_dropper_goal_handle = NULL;
					break;
				case RCLC_RET_ACTION_WAIT_RESULT_REQUEST:
					// we are waiting for the result to be requested.
					break;
				case RCL_RET_ERROR:
					RCSOFTCHECK(rc);
					break;
				default:
					break;
			}
		}
	}

	if (change_claw_state_goal_handle != NULL) {
 		riptide_msgs2__action__ChangeClawState_SendGoal_Request * req =
    		(riptide_msgs2__action__ChangeClawState_SendGoal_Request *) change_claw_state_goal_handle->ros_goal_request;

		enum claw_state claw_state = claw_get_state();
		bool claw_open_param = req->goal.clawopen;

		bool goal_completed;
		if(claw_open_param) {
			goal_completed = claw_state == CLAW_STATE_OPENED;
		} else {
			goal_completed = claw_state == CLAW_STATE_CLOSED;
		}

		if(goal_completed) {
			LOG_DEBUG("claw change state goal finished");
			rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
        	riptide_msgs2__action__ChangeClawState_GetResult_Response response = {0};

			rcl_ret_t rc = rclc_action_send_result(change_claw_state_goal_handle, goal_state, &response);

			switch(rc) {
				case RCL_RET_OK:
					LOG_DEBUG("Change Claw State goal finished and result accepted.");
					change_claw_state_goal_handle = NULL;
					break;
				case RCLC_RET_ACTION_WAIT_RESULT_REQUEST:
					// we are waiting for the result to be requested.
					break;
				case RCL_RET_ERROR:
					RCSOFTCHECK(rc);
					break;
				default:
					break;
			}
		}
	}

	if (actuate_torpedo_goal_handle != NULL) {
 		riptide_msgs2__action__ActuateTorpedos_SendGoal_Request * req =
    		(riptide_msgs2__action__ActuateTorpedos_SendGoal_Request *) actuate_torpedo_goal_handle->ros_goal_request;

		uint8_t torpedo_id = req->goal.torpedo_id;
		enum torpedo_state torpedo_state = torpedo_get_state(torpedo_id);

		if(torpedo_state == TORPEDO_STATE_DISARMED) {
			LOG_DEBUG("actuate torpedos state goal finished");
			rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
        	riptide_msgs2__action__ActuateTorpedos_GetResult_Response response = {0};

			rcl_ret_t rc = rclc_action_send_result(actuate_torpedo_goal_handle, goal_state, &response);

			switch(rc) {
				case RCL_RET_OK:
					LOG_DEBUG("Actuate torpedos goal finished and result accepted.");

					// TODO Finish firing on torpedo state machine

					actuate_torpedo_goal_handle = NULL;
					break;
				case RCLC_RET_ACTION_WAIT_RESULT_REQUEST:
					// we are waiting for the result to be requested.
					break;
				case RCL_RET_ERROR:
					RCSOFTCHECK(rc);
					break;
				default:
					break;
			}
		}
	}

	if (arm_torpedo_dropper_goal_handle != NULL) {
 		riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request * req =
    		(riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *) arm_torpedo_dropper_goal_handle->ros_goal_request;
		bool completed = false;

		if(req->goal.arm_torpedos) {
			completed = torpedo_get_armed_state() == ARMED_STATE_DISARMED;
		} else if(req->goal.arm_droppers) {
			completed = dropper_get_armed_state() == ARMED_STATE_DISARMED;
		} else {
			completed = false;
			LOG_WARN("Neither arm dropper or arm toprpedo set to true. Invalid arm torpedo/dropper goal.");
		}

		if(completed) {
			LOG_DEBUG("arm torpedo_dropper state goal finished");
			rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
        	riptide_msgs2__action__ArmTorpedoDropper_GetResult_Response response = {0};

			rcl_ret_t rc = rclc_action_send_result(arm_torpedo_dropper_goal_handle, goal_state, &response);

			switch(rc) {
				case RCL_RET_OK:
					LOG_DEBUG("Arm torpedo/dropper goal finished and result accepted.");

					arm_torpedo_dropper_goal_handle = NULL;
					break;
				case RCLC_RET_ACTION_WAIT_RESULT_REQUEST:
					// we are waiting for the result to be requested.
					break;
				case RCL_RET_ERROR:
					RCSOFTCHECK(rc);
					break;
				default:
					break;
			}
		}
	}
}

void ros_start(const char* namespace) {
    allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "actuator_node", namespace, &support));

	// create executor
	const uint num_executor_tasks = 7 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, num_executor_tasks, &allocator));

	parameter_server_init(&node, &executor);

	change_claw_state_init(&support, &node, &executor);
	actutate_dropper_init(&support, &node, &executor);
	actuate_torpedos_init(&support, &node, &executor);
	arm_torpedo_dropper_init(&support, &node, &executor);

	RCCHECK(rclc_publisher_init(
		&actuator_status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus),
		"state/actuator",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_timer_init_default(
        &goal_timer,
        &support,
        RCL_MS_TO_NS(100),
        ros_update_actions));
	RCCHECK(rclc_timer_init_default(
        &publisher_timer,
        &support,
        RCL_MS_TO_NS(1000),
        ros_update_publisher));

	RCCHECK(rclc_executor_add_timer(&executor, &goal_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer));

	LOG_DEBUG("Connected to ROS");

	// depth_publisher_init(&support, &node, &executor);
	// state_publish_init(&support, &node, &executor);
	// subscriptions_init(&node, &executor);
}

void ros_spin_ms(long ms) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms)));
}

void ros_cleanup(void) {
	parameter_server_fini(&node);
	// depth_publisher_fini(&node);
	// state_publish_fini(&node);
	// subscriptions_fini(&node);

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
	RCCHECK(rcl_timer_fini(&goal_timer));
	RCCHECK(rcl_timer_fini(&publisher_timer));
}