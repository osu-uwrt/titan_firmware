#include <string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>

#include "actuators.h"
#include "ros.h"
#include "ros_internal.h"

#define TORPEDO_SERVICE_NAME "command/torpedo"
#define DROPPER_SERVICE_NAME "command/dropper"
#define CLAW_SERVICE_NAME "command/claw"
#define NOTIFY_RELOAD_SERVICE_NAME "command/notify_reload"
#define ACTUATOR_ARM_SERVICE_NAME "command/actuator_arm"

static rcl_service_t torpedo_service;
static std_srvs__srv__Trigger_Request torpedo_req;
static std_srvs__srv__Trigger_Response torpedo_res;

static rcl_service_t dropper_service;
static std_srvs__srv__Trigger_Request dropper_req;
static std_srvs__srv__Trigger_Response dropper_res;

static rcl_service_t notify_reload_service;
static std_srvs__srv__Trigger_Request notify_reload_req;
static std_srvs__srv__Trigger_Response notify_reload_res;

static rcl_service_t claw_service;
static std_srvs__srv__SetBool_Request claw_req;
static std_srvs__srv__SetBool_Response claw_res;

static rcl_service_t actuator_arm_service;
static std_srvs__srv__SetBool_Request actuator_arm_req;
static std_srvs__srv__SetBool_Response actuator_arm_res;

static void torpedo_service_callback(__unused const void * req, void * res) {
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;

    const char* message = "";

    res_in->success = torpedo_fire(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char*) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len+1;   // Add null terminated byte
}

static void dropper_service_callback(__unused const void * req, void * res) {
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;

    const char* message = "";

    res_in->success = dropper_drop_marker(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char*) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len+1;   // Add null terminated byte
}

static void notify_reload_service_callback(__unused const void * req, void * res) {
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;

    const char* message = "";

    res_in->success = torpedo_notify_reload(&message);
    if (res_in->success) {
        res_in->success = dropper_notify_reload(&message);
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char*) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len+1;   // Add null terminated byte
}

static void claw_service_callback(const void * req, void * res) {
    std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;

    const char* message = "";

    // True, open claw
    if (req_in->data) {
        res_in->success = claw_open(&message);
    }
    // False, close claw
    else {
        res_in->success = claw_close(&message);
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char*) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len+1;   // Add null terminated byte
}

static void actuator_arm_service_callback(const void * req, void * res) {
    std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;

    const char* message = "";

    // True, arm
    if (req_in->data) {
        res_in->success = actuators_arm(&message);
    }
    // False, disarm
    else {
        actuators_disarm();
        res_in->success = true;
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char*) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len+1;   // Add null terminated byte
}

// Define the number of executor handles required for this file
const size_t ros_actuators_num_executor_handles = 5;

rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node, __unused rclc_support_t *support)
{
    // Torpedos
    RCRETCHECK(rclc_service_init_default(&torpedo_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), TORPEDO_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &torpedo_service, &torpedo_req, &torpedo_res, torpedo_service_callback));

    // Droppers
    RCRETCHECK(rclc_service_init_default(&dropper_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), DROPPER_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &dropper_service, &dropper_req, &dropper_res, dropper_service_callback));

    // Notify Reload
    RCRETCHECK(rclc_service_init_default(&notify_reload_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), NOTIFY_RELOAD_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &notify_reload_service, &notify_reload_req, &notify_reload_res, notify_reload_service_callback));

    // Claw
    RCRETCHECK(rclc_service_init_default(&claw_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), CLAW_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &claw_service, &claw_req, &claw_res, claw_service_callback));

    // Actuator Arm
    RCRETCHECK(rclc_service_init_default(&actuator_arm_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), ACTUATOR_ARM_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &actuator_arm_service, &actuator_arm_req, &actuator_arm_res, actuator_arm_service_callback));

    return RCL_RET_OK;
}

void ros_actuators_fini(rcl_node_t *node)
{
    RCSOFTCHECK(rcl_service_fini(&torpedo_service, node));
    RCSOFTCHECK(rcl_service_fini(&dropper_service, node));
    RCSOFTCHECK(rcl_service_fini(&notify_reload_service, node));
    RCSOFTCHECK(rcl_service_fini(&claw_service, node));
    RCSOFTCHECK(rcl_service_fini(&actuator_arm_service, node));
}