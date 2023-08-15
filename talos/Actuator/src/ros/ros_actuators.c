#include "actuators.h"
#include "ros.h"
#include "ros_internal.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <riptide_msgs2/msg/actuator_status.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>

#include <string.h>

static rcl_publisher_t status_publisher;
static rcl_publisher_t busy_publisher;

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

static rcl_service_t arm_service;
static std_srvs__srv__SetBool_Request actuator_arm_req;
static std_srvs__srv__SetBool_Response actuator_arm_res;

// ========================================
// Status Publishing
// ========================================

rcl_ret_t ros_actuators_update_status(void) {
    std_msgs__msg__Bool busy_msg = { .data = actuators_get_busy() };
    RCRETCHECK(rcl_publish(&busy_publisher, &busy_msg, NULL));

    riptide_msgs2__msg__ActuatorStatus status_msg;
    status_msg.claw_state = claw_get_state();
    status_msg.torpedo_state = torpedo_get_state();
    status_msg.torpedo_available_count = torpedo_get_available();
    status_msg.dropper_state = dropper_get_state();
    status_msg.dropper_available_count = dropper_get_available();

    RCRETCHECK(rcl_publish(&status_publisher, &status_msg, NULL));

// Publish dynamixel status if running v2 actuators
#if ACTUATOR_V2_SUPPORT
    RCRETCHECK(actuator_v2_dynamixel_update_status());
#endif

    return RCL_RET_OK;
}

// ========================================
// Service Callbacks
// ========================================

static void torpedo_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = torpedo_fire(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void dropper_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = dropper_drop_marker(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void notify_reload_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = torpedo_notify_reload(&message);
    if (res_in->success) {
        res_in->success = dropper_notify_reload(&message);
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void claw_service_callback(const void *req, void *res) {
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    const char *message = "";

    // True, open claw
    if (req_in->data) {
        res_in->success = claw_open(&message);
    }
    // False, close claw
    else {
        res_in->success = claw_close(&message);
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void arm_service_callback(const void *req, void *res) {
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    const char *message = "";

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
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

// ========================================
// Initialization
// ========================================

// Define the number of executor handles required for this file
const size_t ros_actuators_num_executor_handles = 5;

rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node) {
    // Torpedos
    RCRETCHECK(rclc_service_init_default(&torpedo_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         TORPEDO_SERVICE_NAME));
    RCRETCHECK(
        rclc_executor_add_service(executor, &torpedo_service, &torpedo_req, &torpedo_res, torpedo_service_callback));

    // Droppers
    RCRETCHECK(rclc_service_init_default(&dropper_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         DROPPER_SERVICE_NAME));
    RCRETCHECK(
        rclc_executor_add_service(executor, &dropper_service, &dropper_req, &dropper_res, dropper_service_callback));

    // Notify Reload
    RCRETCHECK(rclc_service_init_default(
        &notify_reload_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), NOTIFY_RELOAD_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &notify_reload_service, &notify_reload_req, &notify_reload_res,
                                         notify_reload_service_callback));

    // Claw
    RCRETCHECK(rclc_service_init_default(&claw_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                         CLAW_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &claw_service, &claw_req, &claw_res, claw_service_callback));

    // Actuator Arm
    RCRETCHECK(rclc_service_init_default(&arm_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                         ARM_SERVICE_NAME));
    RCRETCHECK(
        rclc_executor_add_service(executor, &arm_service, &actuator_arm_req, &actuator_arm_res, arm_service_callback));

    // State Publishers
    RCRETCHECK(rclc_publisher_init_best_effort(&busy_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                               BUSY_TOPIC_NAME));
    RCRETCHECK(rclc_publisher_init_best_effort(
        &status_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus), STATUS_TOPIC_NAME));

    return RCL_RET_OK;
}

void ros_actuators_fini(rcl_node_t *node) {
    RCSOFTCHECK(rcl_publisher_fini(&status_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&busy_publisher, node));
    RCSOFTCHECK(rcl_service_fini(&torpedo_service, node));
    RCSOFTCHECK(rcl_service_fini(&dropper_service, node));
    RCSOFTCHECK(rcl_service_fini(&notify_reload_service, node));
    RCSOFTCHECK(rcl_service_fini(&claw_service, node));
    RCSOFTCHECK(rcl_service_fini(&arm_service, node));
}
