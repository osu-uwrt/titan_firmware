#include "actuators/ros_claw.h"

#include "actuators/actuator.h"
#include "actuators/hiwonder_driver.h"

#include "titan/logger.h"

#include <riptide_msgs2/msg/actuator_status.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

#define ARM_SUBSCRIPTION_NAME "command/actuator/arm"
#define MOVE_TIME_SUBSCRIPTION_NAME "command/actuator/claw_move_s"
#define DEGREE_PUBLISHER_NAME "state/actuator/degrees"
#define STATUS_TOPIC_NAME "state/actuator/status"
#define ACTUATOR_FEEDBACK_MSG_TOPIC_NAME "state/actuator/cmd_feedback"
#define ACTUATOR_FEEDBACK_STATE_TOPIC_NAME "state/actuator/cmd_status"

// ROS objects
static rcl_subscription_t arm_subscription;
static std_msgs__msg__Bool actuator_arm_msg;

static rcl_publisher_t status_publisher;

static rcl_publisher_t cmd_feedback_publisher;
static rcl_publisher_t cmd_status_publisher;

static std_msgs__msg__String cmd_feedback;
static std_msgs__msg__Bool cmd_status;
static bool new_cmd = false;

// Debug ROS interface
static rcl_subscription_t move_time_subscription;
static std_msgs__msg__Float32 move_time_msg;
static rcl_publisher_t degree_publisher;

// Servo objects
#define CLAW_SERVO_ID 3
#define CLAW_HOME_DEG 0

servo_t claw_servo;

#define DEBUG_MOVE_TIME_MS 1000
#define SERVO_MAX_SPEED 1000

// ========================================
// Actuator Functions
// ========================================

// uint8_t claw_grip_get_state() {
//     if (!claw_grip.connected) {
//         return riptide_msgs2__msg__ActuatorStatus__CLAW_ERROR;
//     }
//     else if (!claw_grip.enabled) {
//         return riptide_msgs2__msg__ActuatorStatus__CLAW_DISARMED;
//     }
//     else {
//         return riptide_msgs2__msg__ActuatorStatus__CLAW_SPOON_CLOSED;
//     }
// }

static bool actuators_arm(const char **errMsgOut) {
    // Make sure a kill switch interrupt won't fire in between checking conditions and arming
    // uint32_t prev_interrupts = save_and_disable_interrupts();

    // Don't allow arming if killed
    if (safety_kill_get_asserting_kill()) {
        // restore_interrupts(prev_interrupts);
        *errMsgOut = "Kill Switch Removed";
        return false;
    }

    bool return_code = true;

    // Don't allow arming if already armed
    if (claw_servo.enabled) {
        // restore_interrupts(prev_interrupts);
        *errMsgOut = "Claw actuator already armed";
        // return false;
        return_code = false;
    }
    else {
        LOG_INFO("Arming claw actuator");

        // We're good to arm
        servo_set_armed(&claw_servo, true);
    }
    // restore_interrupts(prev_interrupts);

    // Perform individual arm actions for actuators
    // if (!torpedo_arm(errMsgOut)) {
    //     return false;
    // }

    // if (!dropper_notify_reload(errMsgOut)) {
    //     return false;
    // }

    return return_code;
}

static bool claw_move_time(const char **errMsgOut, int32_t time_ms) {
    servo_continuous_move_ms(&claw_servo, SERVO_MAX_SPEED * (time_ms / abs(time_ms)), abs(time_ms));
    return true;
}

void claw_ping_servo() {
    servo_ping(&claw_servo);
}

// ========================================
// Status Publishing
// ========================================

// rcl_ret_t ros_actuators_update_status() {
//     // std_msgs__msg__Bool busy_msg = { .data = move_active };
//     // RCRETCHECK(rcl_publish(&busy_publisher, &busy_msg, NULL));

//     riptide_msgs2__msg__ActuatorStatus status_msg;
//     // status_msg.actuators_armed = enabled;
//     // status_msg.claw_state = 0;
//     // status_msg.torpedo_state = torpedo_get_state();
//     // status_msg.torpedo_available_count = num_torp;
//     // status_msg.dropper_state = dropper_get_state();
//     // status_msg.dropper_available_count = num_marker;

//     status_msg.claw_state = claw_grip_get_state();

//     RCRETCHECK(rcl_publish(&status_publisher, &status_msg, NULL));

//     // Publish dynamixel status if running v2 actuators
//     // #if ACTUATOR_V2_SUPPORT
//     //     RCRETCHECK(actuator_v2_dynamixel_update_status());
//     // #endif

//     return RCL_RET_OK;
// }

// rcl_ret_t ros_actuators_update_cmd_feedback() {
//     if (!new_cmd)
//         return RCL_RET_OK;

//     RCRETCHECK(rcl_publish(&cmd_feedback_publisher, &cmd_feedback, NULL));
//     RCRETCHECK(rcl_publish(&cmd_status_publisher, &cmd_status, NULL));

//     new_cmd = false;
//     return RCL_RET_OK;
// }

// rcl_ret_t ros_update_actuator_degrees() {
//     servo_read_deg(&claw_grip);

//     // This read is almost definitely not the new data requested above, but that's fine
//     std_msgs__msg__Int32 deg_msg;
//     deg_msg.data = claw_grip.curr_deg;

//     RCRETCHECK(rcl_publish(&degree_publisher, &deg_msg, NULL));

//     return RCL_RET_OK;
// }

// ========================================
// Subscription Callbacks
// ========================================

static void arm_subscription_callback(const void *msgin) {
    std_msgs__msg__Bool *msg = (std_msgs__msg__Bool *) msgin;

    const char *message = "";

    // True, arm
    if (msg->data) {
        cmd_status.data = actuators_arm(&message);
    }
    // False, disarm
    else {
        servo_set_armed(&claw_servo, false);

        cmd_status.data = true;
    }

    size_t msg_len = strlen(message);
    cmd_feedback.data.data = (char *) message;
    cmd_feedback.data.size = msg_len;
    cmd_feedback.data.capacity = msg_len + 1;  // Add null termination byte

    new_cmd = true;
}

static void move_time_subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *) msgin;

    const char *message = "";
    cmd_status.data = claw_move_time(&message, msg->data * 1000.0f);

    size_t msg_len = strlen(message);
    cmd_feedback.data.data = (char *) message;
    cmd_feedback.data.size = msg_len;
    cmd_feedback.data.capacity = msg_len + 1;  // Add null termination byte

    new_cmd = true;
}

// ========================================
// Initialization
// ========================================

// Define the number of executor handles required for this file
const size_t ros_claw_num_executor_handles = 2;

rcl_ret_t ros_claw_init(rclc_executor_t *executor, rcl_node_t *node) {
    // General
    RCRETCHECK(rclc_subscription_init_default(&arm_subscription, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                              ARM_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &arm_subscription, &actuator_arm_msg, arm_subscription_callback,
                                              ON_NEW_DATA));

    RCRETCHECK(rclc_publisher_init_best_effort(
        &status_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, ActuatorStatus), STATUS_TOPIC_NAME));

    // Claw
    RCRETCHECK(rclc_subscription_init_default(&move_time_subscription, node,
                                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                              MOVE_TIME_SUBSCRIPTION_NAME));
    RCRETCHECK(rclc_executor_add_subscription(executor, &move_time_subscription, &move_time_msg,
                                              move_time_subscription_callback, ON_NEW_DATA));

    // Command Feedback Pubishers
    RCRETCHECK(rclc_publisher_init_default(&cmd_feedback_publisher, node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                           ACTUATOR_FEEDBACK_MSG_TOPIC_NAME));

    RCRETCHECK(rclc_publisher_init_default(&cmd_status_publisher, node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                           ACTUATOR_FEEDBACK_STATE_TOPIC_NAME));

    return RCL_RET_OK;
}

rcl_ret_t ros_claw_fini(rcl_node_t *node) {
    RCSOFTCHECK(rcl_subscription_fini(&arm_subscription, node));
    RCSOFTCHECK(rcl_subscription_fini(&move_time_subscription, node));
    RCSOFTCHECK(rcl_publisher_fini(&status_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&degree_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&cmd_feedback_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&cmd_status_publisher, node));

    return RCL_RET_OK;
}

void claw_init_servo() {
    make_servo(&claw_servo, CLAW_SERVO_ID, CLAW_HOME_DEG);
}
