#include "actuators.h"
#include "ros_internal.h"
#include "safety_interface.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/dynamixel_status.h>
#include <std_srvs/srv/trigger.h>

static rcl_publisher_t dynamixel_status_publisher;

static rcl_service_t move_home_service;
static std_srvs__srv__Trigger_Request move_home_req;
static std_srvs__srv__Trigger_Response move_home_res;

static rcl_service_t set_home_service;
static std_srvs__srv__Trigger_Request set_home_req;
static std_srvs__srv__Trigger_Response set_home_res;

static rcl_service_t set_closed_pos_service;
static std_srvs__srv__Trigger_Request set_closed_pos_req;
static std_srvs__srv__Trigger_Response set_closed_pos_res;

rcl_ret_t actuator_v2_dynamixel_update_status(void) {
    riptide_msgs2__msg__DynamixelStatus status;
    actuator_dxlitr_t itr;

    // Iterate through dynamixels, publishing status
    actuator_dxlitr_init(&itr);
    while (actuator_dxlitr_next(&itr, &status)) {
        RCRETCHECK(rcl_publish(&dynamixel_status_publisher, &status, NULL));
    }

    return RCL_RET_OK;
}

static void move_home_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = torpedo_marker_move_home(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void set_home_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = torpedo_marker_set_home(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

static void set_closed_pos_service_callback(__unused const void *req, void *res) {
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *) res;

    const char *message = "";

    res_in->success = claw_set_closed_position(&message);

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

const size_t actuator_v2_dynamixel_num_executor_handles = 3;

rcl_ret_t actuator_v2_dynamixel_init(rcl_node_t *node, rclc_executor_t *executor) {
    RCRETCHECK(rclc_publisher_init_best_effort(&dynamixel_status_publisher, node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, DynamixelStatus),
                                               DYNAMIXEL_STATUS_TOPIC_NAME));

    RCRETCHECK(rclc_service_init_default(&move_home_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         TORPEDO_MARKER_MOVE_HOME_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &move_home_service, &move_home_req, &move_home_res,
                                         move_home_service_callback));

    RCRETCHECK(rclc_service_init_default(&set_home_service, node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         TORPEDO_MARKER_SET_HOME_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &set_home_service, &set_home_req, &set_home_res,
                                         set_home_service_callback));

    RCRETCHECK(rclc_service_init_default(&set_closed_pos_service, node,
                                         ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                         CLAW_SET_CLOSED_POS_SERVICE_NAME));
    RCRETCHECK(rclc_executor_add_service(executor, &set_closed_pos_service, &set_closed_pos_req, &set_closed_pos_res,
                                         set_closed_pos_service_callback));

    return RCL_RET_OK;
}

void actuator_v2_dynamixel_fini(rcl_node_t *node) {
    RCSOFTCHECK(rcl_publisher_fini(&dynamixel_status_publisher, node));
    RCSOFTCHECK(rcl_service_fini(&move_home_service, node));
    RCSOFTCHECK(rcl_service_fini(&set_home_service, node));
    RCSOFTCHECK(rcl_service_fini(&set_closed_pos_service, node));
}
