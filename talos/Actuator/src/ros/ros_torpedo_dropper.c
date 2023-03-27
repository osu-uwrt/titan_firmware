#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "basic_logger/logging.h"
#include <riptide_msgs2/action/arm_torpedo_dropper.h>
#include <riptide_msgs2/action/actuate_droppers.h>
#include <riptide_msgs2/action/actuate_torpedos.h>

#include "ros/ros.h"
#include "actuators/torpedo.h"
#include "actuators/dropper.h"
#include "actuators/arm_state.h"

#include <assert.h> // TODO remove

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

static rclc_action_server_t arm_torpedo_dropper_server;
static rclc_action_goal_handle_t *arm_torpedo_dropper_goal_handle = NULL;
static riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request arm_torpedo_dropper_ros_goal_request[10];

static rclc_action_server_t actuate_dropper_server;
static rclc_action_goal_handle_t *actuate_dropper_goal_handle = NULL;
static riptide_msgs2__action__ActuateDroppers_SendGoal_Request actuate_droppers_ros_goal_request[10];

static rclc_action_server_t actuate_torpedo_server;
static rclc_action_goal_handle_t *actuate_torpedo_goal_handle = NULL;
static riptide_msgs2__action__ActuateTorpedos_SendGoal_Request actuate_torpedo_ros_goal_request[10];

rcl_ret_t arm_torpedo_dropper_goal_request(rclc_action_goal_handle_t *goal_handle, void *context)
{
    (void)context;

    riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *req =
        (riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *)goal_handle->ros_goal_request;

    if (arm_torpedo_dropper_goal_handle != NULL)
    {
        LOG_WARN("arm torpedo dropper goal already running. Rejecting new goal.");
        return RCL_RET_ACTION_GOAL_REJECTED;
    }

    bool result;
    if (req->goal.arm_droppers)
    {
        result = dropper_arm();
        assert(false);
    }
    else if (req->goal.arm_torpedos)
    {
        result = torpedo_arm();
    }
    else
    {
        result = false;
        LOG_DEBUG("Neither arm dropper or arm toprpedo set to true. Rejecting arm torpedo/dropper goal.");
    }

    if (result)
    {
        LOG_DEBUG("Arm dropper/torpedo worked. Accepted arm torpedo/dropper goal");
        arm_torpedo_dropper_goal_handle = goal_handle;
        return RCL_RET_ACTION_GOAL_ACCEPTED;
    }
    else
    {
        LOG_DEBUG("Arm dropper/torpedo failed. Rejecting arm torpedo/dropper goal");
        return RCL_RET_ACTION_GOAL_REJECTED;
    }
}

bool arm_torpedo_dropper_cancel(rclc_action_goal_handle_t *goal_handle, void *context)
{
    (void)context;
    (void)goal_handle;
    riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *req =
        (riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request *)goal_handle->ros_goal_request;

    bool cancel;
    if (req->goal.arm_torpedos)
    {
        assert(false);
        cancel = torpedo_disarm();
    }
    else if (req->goal.arm_droppers)
    {
        assert(false);
        cancel = dropper_disarm();
    }
    else
    {
        cancel = false;
        LOG_WARN("Neither arm dropper or arm toprpedo set to true. Invalid arm torpedo/dropper goal.");
    }

    if (cancel)
    {
        LOG_DEBUG("cancelling arm torpedo/dropper goal.");
        arm_torpedo_dropper_goal_handle = NULL;
    }

    return cancel;
}

rcl_ret_t actuate_torpedos_goal_request(rclc_action_goal_handle_t * goal_handle, void * context) {
	  (void) context;

 	riptide_msgs2__action__ActuateTorpedos_SendGoal_Request * req =
    	(riptide_msgs2__action__ActuateTorpedos_SendGoal_Request *) goal_handle->ros_goal_request;

	if (actuate_torpedo_goal_handle != NULL) {
		LOG_WARN("Actuate torpedo goal already running. Rejecting new goal.");
		return RCL_RET_ACTION_GOAL_REJECTED;
	}

	uint8_t torpedo = req->goal.torpedo_id;

    assert(false);
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

rcl_ret_t ros_torpedo_dropper_init(rclc_executor_t *executor, rcl_node_t *node, rclc_support_t *support)
{
    RCRETCHECK(
        rclc_action_server_init_default(
            &arm_torpedo_dropper_server,
            node,
            support,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ArmTorpedoDropper),
            "arm_torpedo_dropper"));

    RCRETCHECK(rclc_executor_add_action_server(
        executor,
        &arm_torpedo_dropper_server,
        1,
        arm_torpedo_dropper_ros_goal_request,
        sizeof(riptide_msgs2__action__ArmTorpedoDropper_SendGoal_Request),
        arm_torpedo_dropper_goal_request,
        arm_torpedo_dropper_cancel,
        (void *)&arm_torpedo_dropper_server));

    RCRETCHECK(
        rclc_action_server_init_default(
        &actuate_torpedo_server,
        node,
        support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ActuateTorpedos),
        "actuate_torpedo"
    ));

	RCRETCHECK(rclc_executor_add_action_server(
        executor,
        &actuate_torpedo_server,
        1,
        actuate_torpedo_ros_goal_request,
        sizeof(riptide_msgs2__action__ActuateTorpedos_SendGoal_Request),
        actuate_torpedos_goal_request,
        actuate_torpedos_cancel,
        (void *) &actuate_torpedo_server));

    RCRETCHECK(
        rclc_action_server_init_default(
        &actuate_dropper_server,
        node,
        support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(riptide_msgs2, ActuateDroppers),
        "actuate_dropper"
    ));

	RCRETCHECK(rclc_executor_add_action_server(
        executor,
        &actuate_dropper_server,
        1,
        actuate_droppers_ros_goal_request,
        sizeof(riptide_msgs2__action__ActuateDroppers_SendGoal_Request),
        actuate_dropper_goal_request,
        actuate_dropper_cancel,
        (void *) &actuate_dropper_server));

    return RCL_RET_OK;
}

void ros_torpedo_dropper_fini(rcl_node_t *node)
{
    RCSOFTCHECK(rclc_action_server_fini(&arm_torpedo_dropper_server, node));
    RCSOFTCHECK(rclc_action_server_fini(&actuate_dropper_server, node));
    RCSOFTCHECK(rclc_action_server_fini(&actuate_torpedo_server, node));
}