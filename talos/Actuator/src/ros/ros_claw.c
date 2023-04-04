#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "basic_logger/logging.h"

#include <riptide_msgs2/action/change_claw_state.h>

#include "ros/ros.h"
#include "actuators/claw.h"

#include <assert.h> // TODO remove

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

static rclc_action_server_t change_claw_state_server;
static rclc_action_goal_handle_t *change_claw_state_goal_handle = NULL;
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
		assert(false);
		result = claw_open();
	} else {
		assert(false);
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

rcl_ret_t ros_claw_init(rclc_executor_t *executor, rcl_node_t *node, rclc_support_t *support)
{
    RCRETCHECK(
        rclc_action_server_init_default(
            &change_claw_state_server,
            node,
            support,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ChangeClawState),
            "change_claw_state"));

    RCRETCHECK(rclc_executor_add_action_server(
        executor,
        &change_claw_state_server,
        1,
        change_claw_state_ros_goal_request,
        sizeof(riptide_msgs2__action__ChangeClawState_SendGoal_Request),
        change_claw_state_goal_request,
        change_claw_state_cancel,
        (void *)&change_claw_state_server));

    return RCL_RET_OK;
}

void ros_claw_fini(rcl_node_t *node) {
	RCSOFTCHECK(rclc_action_server_fini(&change_claw_state_server, node));
}