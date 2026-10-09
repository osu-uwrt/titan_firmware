#include "actuators/ros_actuators.h"

#include "actuators/actuator.h"
#include "actuators/claw.h"
#include "actuators/hiwonder_driver.h"

#include "titan/logger.h"

#include <riptide_msgs2/msg/actuator_status.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <string.h>
#include "hardware/sync.h"

#define POSITION_SUBSCRIPTION_NAME "command/actuator/claw/position"
#define TARE_SUBSCRIPTION_NAME "command/actuator/claw/tare"
#define RAW_POSITION_TOPIC_NAME "state/actuator/claw/raw_position"
#define VOLTAGE_TOPIC_NAME "state/actuator/claw/voltage_raw"
#define ARM_SUBSCRIPTION_NAME "command/actuator/arm"
#define CLAW_ARM_SUBSCRIPTION_NAME "command/actuator/claw/arm"
#define ACTUATOR_FEEDBACK_MSG_TOPIC_NAME "state/actuator/cmd_feedback"
#define ACTUATOR_FEEDBACK_STATE_TOPIC_NAME "state/actuator/cmd_status"
#define DEGREE_PUBLISHER_NAME "state/actuator/degrees"
#define STATUS_TOPIC_NAME "state/actuator/status"
// ROS objects
static rcl_publisher_t status_publisher;
static rcl_subscription_t arm_subscription;
static rcl_subscription_t claw_arm_subscription;
static rcl_subscription_t position_subscription;
static rcl_subscription_t tare_subscription;
static std_msgs__msg__Int32 position_msg;
static std_msgs__msg__Empty tare_msg;
static rcl_publisher_t raw_position_publisher;
static rcl_publisher_t voltage_publisher;
static std_msgs__msg__Bool actuator_arm_msg;
static std_msgs__msg__Bool claw_arm_msg;
static rcl_publisher_t cmd_feedback_publisher;
static rcl_publisher_t cmd_status_publisher;
static std_msgs__msg__String cmd_feedback;
static std_msgs__msg__Bool cmd_status;
static bool new_cmd;


// Debug ROS interface
static rcl_publisher_t degree_publisher;

// Servo objects
static servo_t *claw_grip;
servo_t claw_rack;


// ========================================
// Actuator Functions
// ========================================

uint8_t claw_grip_get_state() {
    if (!claw_grip->connected) {
        return riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
    }
    else if (!claw_grip->enabled) {
        return riptide_msgs2__msg__ActuatorStatus__CLAW_DISARMED;
    }
    else {
        return riptide_msgs2__msg__ActuatorStatus__CLAW_SPOON_CLOSED;
    }
}

// ========================================
// Status Publishing
// ========================================

rcl_ret_t ros_actuators_update_status() {
    // std_msgs__msg__Bool busy_msg = { .data = move_active };
    // RCRETCHECK(rcl_publish(&busy_publisher, &busy_msg, NULL));

    riptide_msgs2__msg__ActuatorStatus status_msg = { 0 };
    status_msg.actuators_armed = claw_grip->desired_armed_state && claw_grip->enabled &&
                                claw_rack.desired_armed_state && claw_rack.enabled;
    // status_msg.claw_state = 0;
    // status_msg.torpedo_state = torpedo_get_state();
    // status_msg.torpedo_available_count = num_torp;
    // status_msg.dropper_state = dropper_get_state();
    // status_msg.dropper_available_count = num_marker;

    status_msg.claw_state = claw_grip_get_state();

    RCRETCHECK(rcl_publish(&status_publisher, &status_msg, NULL));

    // Publish dynamixel status if running v2 actuators
    // #if ACTUATOR_V2_SUPPORT
    //     RCRETCHECK(actuator_v2_dynamixel_update_status());
    // #endif

    return RCL_RET_OK;
}

rcl_ret_t ros_update_actuator_degrees() {
    std_msgs__msg__Int32 deg_msg;
    deg_msg.data = claw_grip->curr_deg;

    RCRETCHECK(rcl_publish(&degree_publisher, &deg_msg, NULL));

    return RCL_RET_OK;
}

rcl_ret_t ros_actuators_update_claw_telemetry(void) {
    int16_t raw;
    uint16_t voltage;
    if (claw_get_raw_position(&raw)) {
        std_msgs__msg__Int32 msg = { .data = raw };
        RCRETCHECK(rcl_publish(&raw_position_publisher, &msg, NULL));
    }
    if (claw_get_voltage(&voltage)) {
        std_msgs__msg__Int32 msg = { .data = voltage };
        RCRETCHECK(rcl_publish(&voltage_publisher, &msg, NULL));
    }
    return RCL_RET_OK;
}

static void set_cmd_feedback(bool accepted, const char *message) {
    cmd_status.data = accepted;
    cmd_feedback.data.data = (char *) message;
    cmd_feedback.data.size = strlen(message);
    cmd_feedback.data.capacity = cmd_feedback.data.size + 1;
    new_cmd = true;
}

static void position_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = msgin;
    bool accepted = claw_set_position(msg->data);
    set_cmd_feedback(accepted, claw_get_position_feedback());
}

static void tare_subscription_callback(__unused const void *msgin) {
    bool accepted = claw_tare();
    set_cmd_feedback(accepted, accepted ? "Claw closed reference set" :
        "Claw tare rejected: requires fresh feedback and no active move or pending stop");
}

rcl_ret_t ros_actuators_update_cmd_feedback(void) {
    if (!new_cmd)
        return RCL_RET_OK;
    RCRETCHECK(rcl_publish(&cmd_feedback_publisher, &cmd_feedback, NULL));
    RCRETCHECK(rcl_publish(&cmd_status_publisher, &cmd_status, NULL));
    new_cmd = false;
    return RCL_RET_OK;
}

static void arm_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = msgin;
    const char *message = "";
    cmd_status.data = msg->data;
    if (!msg->data)
        claw_stop();
    servo_set_armed(claw_get_servo(), msg->data);
    servo_set_armed(&claw_rack, msg->data);

    set_cmd_feedback(cmd_status.data, message);
}

static void claw_arm_subscription_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = msgin;
    if (!msg->data)
        claw_stop();
    servo_set_armed(claw_get_servo(), msg->data);

    set_cmd_feedback(true, msg->data ? "Claw arm requested" : "Claw disarm requested");
}

// ========================================
// Initialization
// ========================================

// Define the number of executor handles required for this file
const size_t ros_actuators_num_executor_handles = 4;

rcl_ret_t ros_actuators_init(rclc_executor_t *executor, rcl_node_t *node) {
    RCRETCHECK(rclc_subscription_init_default(&position_subscription, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), POSITION_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &position_subscription, &position_msg,
        position_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_subscription_init_default(&tare_subscription, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), TARE_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &tare_subscription, &tare_msg,
        tare_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_publisher_init_best_effort(&raw_position_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), RAW_POSITION_TOPIC_NAME));
    RCRETCHECK(rclc_publisher_init_best_effort(&voltage_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), VOLTAGE_TOPIC_NAME));
    RCRETCHECK(rclc_subscription_init_default(&arm_subscription, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ARM_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &arm_subscription, &actuator_arm_msg,
        arm_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_subscription_init_default(&claw_arm_subscription, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), CLAW_ARM_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &claw_arm_subscription, &claw_arm_msg,
        claw_arm_subscription_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_publisher_init_default(&cmd_feedback_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), ACTUATOR_FEEDBACK_MSG_TOPIC_NAME));
    RCRETCHECK(rclc_publisher_init_default(&cmd_status_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ACTUATOR_FEEDBACK_STATE_TOPIC_NAME));
    RCRETCHECK(rclc_publisher_init_best_effort(
        &status_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus), STATUS_TOPIC_NAME));

    // Claw
    RCRETCHECK(rclc_publisher_init_default(&degree_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                           DEGREE_PUBLISHER_NAME));

    return RCL_RET_OK;
}

rcl_ret_t ros_actuators_fini(rcl_node_t *node) {
    RCSOFTCHECK(rcl_subscription_fini(&position_subscription, node));
    RCSOFTCHECK(rcl_subscription_fini(&tare_subscription, node));
    RCSOFTCHECK(rcl_publisher_fini(&raw_position_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&voltage_publisher, node));
    RCSOFTCHECK(rcl_subscription_fini(&arm_subscription, node));
    RCSOFTCHECK(rcl_subscription_fini(&claw_arm_subscription, node));
    RCSOFTCHECK(rcl_publisher_fini(&cmd_feedback_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&cmd_status_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&status_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&degree_publisher, node));

    return RCL_RET_OK;
}

void init_servos() {
    servo_init_internal();

    claw_init();
    claw_grip = claw_get_servo();
    make_servo(&claw_rack, 4, 0);
}
