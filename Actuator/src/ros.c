#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include "drivers/safety.h"
#include "actuators/dropper.h"

#include "basic_logger/logging.h"
#include "pico_uart_transports.h"

#include <riptide_msgs2/action/actuate_droppers.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_FATAL("Failed status on in " __FILE__ ":%d : %d. Aborting.",__LINE__,(int)temp_rc); panic("Unrecoverable ROS Error");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_SOFT_FAIL);}}

static rcl_allocator_t allocator = {0};
static rclc_support_t support = {0};
static rcl_node_t node = {0};
static rclc_executor_t executor = {0};
static rcl_timer_t timer = {0};

static rclc_action_server_t actuator_dropper_server;

//
// Actuate Dropper Action Server
//

static rclc_action_goal_handle_t * actuate_dropper_goal_handle;

rcl_ret_t actuate_dropper_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) { 
	  (void) context;

 	riptide_msgs2__action__ActuateDroppers_SendGoal_Request * req =
    	(riptide_msgs2__action__ActuateDroppers_SendGoal_Request *) goal_handle->ros_goal_request;

	if (actuate_dropper_goal_handle != NULL) {
		LOG_WARN("Actuate dropper goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	uint8_t dropper = req->goal.dropper_id;
	actuate_dropper_goal_handle = goal_handle;

	bool result = dropper_drop_marker(dropper);

	if(result)  {
		return RCL_RET_ACTION_GOAL_ACCEPTED;
	} else {
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
	
    riptide_msgs2__action__ActuateDroppers_SendGoal_Request ros_goal_request[10];
	RCCHECK(rclc_executor_add_action_server(
        &executor,
        &actuator_dropper_server,
        10,
        ros_goal_request,
        sizeof(riptide_msgs2__action__ActuateDroppers_SendGoal_Request),
        actuate_dropper_goal_request,
        actuate_dropper_cancel,
        (void *) &actuator_dropper_server));
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

		if(dropper_state == DROPPER_STATE_DROPPED) {
			rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
        	riptide_msgs2__action__ActuateDroppers_GetResult_Response response = {0};

			rcl_ret_t rc = rclc_action_send_result(actuate_dropper_goal_handle, goal_state, &response);

			switch(rc) { 
				case RCL_RET_OK:
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

	actutate_dropper_init(&support, &node, &executor);

	rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        ros_update_actions);


	// parameter_server_init(&node, &executor);
	// depth_publisher_init(&support, &node, &executor);
	// state_publish_init(&support, &node, &executor);
	// subscriptions_init(&node, &executor);
}

void ros_spin_ms(long ms) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms)));
}

void ros_cleanup(void) {
	// parameter_server_fini(&node);
	// depth_publisher_fini(&node);
	// state_publish_fini(&node);
	// subscriptions_fini(&node);

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
	RCCHECK(rcl_timer_fini(&timer));
}