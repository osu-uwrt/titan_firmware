#include "actuators/ros_torp.h"

#include "actuators/actuator.h"
#include "actuators/hiwonder_driver.h"
// #include "ros_internal.h"
#include "safety_interface.h"

#include "pico/sync.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <riptide_msgs2/msg/actuator_status.h>
#include <riptide_msgs2/msg/dynamixel_status.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>

#define MAX_MISSSED_HEARTBEATS 7
#define HEARTBEAT_PUBLISHER_NAME "state/fw_heartbeat"
#define FIRMWARE_STATUS_PUBLISHER_NAME "state/firmware"
#define KILLSWITCH_PUBLISHER_NAME "state/kill"
#define SOFT_KILL_SUBSCRIBER_NAME "command/software_kill"
#define ELECTRICAL_READING_NAME "state/electrical"
#define PHYSICAL_KILL_NOTIFY_PUBLISHER_NAME "state/physkill_notify"
#define ELECTRICAL_COMMAND_SUBSCRIBER_NAME "command/electrical"
#define TEMP_STATUS_PUBLISHER_NAME "state/temp/poacboard"
#define HUMIDITY_STATUS_PUBLISHER_NAME "state/humidity/poacboard"
#define AUX_SWITCH_PUBLISHER_NAME "state/aux"
#define BALANCING_FEEDBACK_PUBLISHER_NAME "state/batteries_balanced"

#define BUSY_TOPIC_NAME "state/actuator/busy"
#define STATUS_TOPIC_NAME "state/actuator/status"
#define DYNAMIXEL_STATUS_TOPIC_NAME "state/actuator/dynamixel"
#define TORPEDO_SERVICE_NAME "command/actuator/torpedo"
#define DROPPER_SERVICE_NAME "command/actuator/dropper"
#define CLAW_SERVICE_NAME "command/actuator/claw"
#define NOTIFY_RELOAD_SERVICE_NAME "command/actuator/notify_reload"
#define ARM_SERVICE_NAME "command/actuator/arm"
#define TORPEDO_MARKER_MOVE_HOME_SERVICE_NAME "command/actuator/torpedo_marker/go_home"
#define TORPEDO_MARKER_SET_HOME_SERVICE_NAME "command/actuator/torpedo_marker/set_home"
#define CLAW_SET_CLOSED_POS_SERVICE_NAME "command/actuator/claw/set_closed_pos"

static rcl_publisher_t dynamixel_status_publisher;
static rcl_publisher_t status_publisher;
static rcl_publisher_t busy_publisher;

static rcl_service_t move_home_service;
static std_srvs__srv__Trigger_Request move_home_req;
static std_srvs__srv__Trigger_Response move_home_res;

static rcl_service_t set_home_service;
static std_srvs__srv__Trigger_Request set_home_req;
static std_srvs__srv__Trigger_Response set_home_res;

static rcl_service_t torpedo_service;
static std_srvs__srv__Trigger_Request torpedo_req;
static std_srvs__srv__Trigger_Response torpedo_res;

static rcl_service_t dropper_service;
static std_srvs__srv__Trigger_Request dropper_req;
static std_srvs__srv__Trigger_Response dropper_res;

static rcl_service_t notify_reload_service;
static std_srvs__srv__Trigger_Request notify_reload_req;
static std_srvs__srv__Trigger_Response notify_reload_res;

static rcl_service_t arm_service;
static std_srvs__srv__SetBool_Request actuator_arm_req;
static std_srvs__srv__SetBool_Response actuator_arm_res;

#define TORP_NUMBER 2
#define DROPPER_NUMBER 2

#define TORP_2_DEG 100
#define TORP_1_DEG 200

#define MARKER_2_DEG 50
#define MARKER_1_DEG 0

uint8_t num_torp = TORP_NUMBER;
uint8_t num_marker = DROPPER_NUMBER;
//
// Actuator functions
//
bool torpedo_fire(const char **errMsgOut) {
    // Don't allow firing of torpedos when killed
    // if (safety_kill_get_asserting_kill()) {
    //     *errMsgOut = "Kill Switch Removed";
    //     return false;
    // }

    // Make sure that actuators are armed
    if (!enabled) {
        *errMsgOut = "Not Armed";
        return false;
    }

    // Make sure the next index to fire exists
    if (num_torp <= 0) {
        *errMsgOut = "All Torpedos Fired";
        return false;
    }

    // Make sure we're not already firing a torpedo
    if (move_active) {
        *errMsgOut = "Already Firing";
        return false;
    }

    // Make sure we're charged
    // if (!torpedo_check_charged()) {
    //     *errMsgOut = "Not Charged";
    //     return false;
    // }

    // It should be fine to save this outside of a critical section, as this is only incremented by the firing callback
    // As we already checked that we aren't firing, its fine to just take this as it is
    // uint torpedo_index = next_torpedo_index;

    // Ensure all of the timings are initialized (non-zero)
    // for (int i = 0; i < ACTUATOR_NUM_TORPEDO_TIMINGS; i++) {
    //     if (torpedo_timings[torpedo_index][i] == 0) {
    //         *errMsgOut = "Missing Timings";
    //         return false;
    //     }
    // }

    // Begin the PIO firing sequence
    // (in critical section so an untimely kill won't throw us into an undefined state)
    // uint32_t prev_interrupts = save_and_disable_interrupts();

    // // Make sure that the volatile fields didn't change during other checks while we're in the critical section
    // if (safety_kill_get_asserting_kill() || !enabled) {
    //     restore_interrupts(prev_interrupts);
    //     *errMsgOut = "Killed";
    //     return false;
    // }

    // torpedo_firing = true;
    // gpio_put(torpedo_select_pins[torpedo_index], TORP_SEL_LEVEL_ON);
    // if (!torpedo_fire_sequence(torpedo_pio, torpedo_pio_sm, torpedo_timings[torpedo_index])) {
    //     // If the fire failed, clean up and throw an error
    //     torpedo_firing = false;
    //     gpio_put(torpedo_select_pins[torpedo_index], TORP_SEL_LEVEL_OFF);
    //     restore_interrupts(prev_interrupts);

    //     // Reset the PIO SM in hope that it'll fix next time
    //     torpedo_reset(torpedo_pio, torpedo_pio_sm, torpedo_pio_offset, FIRST_COIL_PIN);

    //     safety_raise_fault(FAULT_ACTUATOR_FAILURE);
    //     *errMsgOut = "PIO Error";
    //     return false;
    // }

    // restore_interrupts(prev_interrupts);

    LOG_INFO("Firing Torpedo %d", num_torp);

    if (num_torp == 2)
        servo_set_deg_then_home(TORP_2_DEG);
    else if (num_torp == 1)
        servo_set_deg_then_home(TORP_1_DEG);

    num_torp--;
    return true;
}

uint8_t torpedo_get_state(void) {
    // hard_assert_if(ACTUATORS, !actuators_initialized);

    // Handle individual torpedo state
    // if (!torpedo_timings_valid) {
    //     return riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
    // }
    if (!connected || !homed) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_ERROR;
    }
    if (!enabled) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_DISARMED;
    }
    else if (move_active) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRING;
    }
    else if (num_torp > 0) {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_CHARGED;
    }
    else /*if (torpedo_check_charged())*/ {
        return riptide_msgs2__msg__ActuatorStatus__TORPEDO_FIRED;
    }
    // else {
    //     return riptide_msgs2__msg__ActuatorStatus__TORPEDO_CHARGING;
    // }
    // TODO: Add error on charge timeout
}

bool torpedo_notify_reload(const char **errMsgOut) {
    if (move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    num_torp = TORP_NUMBER;

    LOG_INFO("Marking torpedos reloaded");
    return true;
}

bool dropper_drop_marker(const char **errMsgOut) {
    if (num_marker == 2) {
        servo_set_deg_then_home(MARKER_2_DEG);
    }
    else if (num_marker == 1) {
        servo_set_deg_then_home(MARKER_1_DEG);
    }
    else {
        *errMsgOut = "All Dropped";
        return false;
    }

    num_marker--;
    return true;
}

uint8_t dropper_get_state(void) {
    if (!connected || !homed) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_ERROR;
    }
    else if (!enabled) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DISARMED;
    }
    else if (move_active) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPING;
    }
    else if (num_marker > 0) {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_READY;
    }
    else {
        return riptide_msgs2__msg__ActuatorStatus__DROPPER_DROPPED;
    }
}

bool dropper_notify_reload(const char **errMsgOut) {
    if (move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    num_marker = DROPPER_NUMBER;

    LOG_INFO("Marking dropper reloaded");
    return true;
}

bool actuators_arm(const char **errMsgOut) {
    // Make sure a kill switch interrupt won't fire in between checking conditions and arming
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Don't allow arming if already armed
    if (enabled) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Already Armed";
        return false;
    }

    // Don't allow arming if killed
    // if (safety_kill_get_asserting_kill()) {
    //     restore_interrupts(prev_interrupts);
    //     *errMsgOut = "Kill Switch Removed";
    //     return false;
    // }

    LOG_INFO("Arming Actuators");

    // We're good to arm
    servo_set_armed(true);
    restore_interrupts(prev_interrupts);

    // Perform individual arm actions for actuators
    // if (!torpedo_arm(errMsgOut)) {
    //     return false;
    // }

    // if (!dropper_notify_reload(errMsgOut)) {
    //     return false;
    // }

    return true;
}

bool torpedo_marker_set_home(const char **errMsgOut) {
    if (!connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    // if (torpedo_marker_state->hardware_err) {
    //     *errMsgOut = "Hardware Error";
    //     return false;
    // }

    if (enabled) {
        *errMsgOut = "Must be disarmed";
        return false;
    }

    // struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(torpedo_marker_state->id);
    // volatile struct dynamixel_ram *ram = dynamixel_get_ram(torpedo_marker_state->id);
    // if (!eeprom || !ram) {
    //     *errMsgOut = "State read error";
    //     return false;
    // }

    // int32_t homing_offset_compensation = eeprom->homing_offset;
    // if (homing_offset_compensation > 1024 || homing_offset_compensation < -1024) {
    //     homing_offset_compensation = 0;
    // }

    // int32_t new_homing_offset = -1 * ((ram->present_position - homing_offset_compensation) - POSITION_HOME) % 4096;
    // if (new_homing_offset > 1024 || new_homing_offset < -1024) {
    //     *errMsgOut = "Homing offset too large";
    //     return false;
    // }

    // dynamixel_set_homing_offset(torpedo_marker_state->id, new_homing_offset);
    // dynamixel_request_eeprom_rescan(torpedo_marker_state->id);

    servo_set_home();

    LOG_INFO("Seting home to current position");

    return true;
}

bool torpedo_marker_move_home(const char **errMsgOut) {
    if (!connected) {
        *errMsgOut = "Not Connected";
        return false;
    }

    // if (torpedo_marker_state->hardware_err) {
    //     *errMsgOut = "Hardware Error";
    //     return false;
    // }

    if (!enabled) {
        *errMsgOut = "Not Armed";
        return false;
    }

    if (move_active) {
        *errMsgOut = "Busy";
        return false;
    }

    // dynamixel_set_target_position(torpedo_marker_state->id, POSITION_HOME);
    servo_go_home();
    LOG_INFO("Manually moving torpedo home");

    return true;
}

// ========================================
// Status Publishing
// ========================================

rcl_ret_t ros_actuators_update_status(void) {
    std_msgs__msg__Bool busy_msg = { .data = move_active };
    RCRETCHECK(rcl_publish(&busy_publisher, &busy_msg, NULL));

    riptide_msgs2__msg__ActuatorStatus status_msg;
    status_msg.actuators_armed = enabled;
    status_msg.claw_state = 0;
    status_msg.torpedo_state = torpedo_get_state();
    status_msg.torpedo_available_count = num_torp;
    status_msg.dropper_state = dropper_get_state();
    status_msg.dropper_available_count = num_marker;

    RCRETCHECK(rcl_publish(&status_publisher, &status_msg, NULL));

    // Publish dynamixel status if running v2 actuators
    // #if ACTUATOR_V2_SUPPORT
    //     RCRETCHECK(actuator_v2_dynamixel_update_status());
    // #endif

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
        servo_set_armed(false);
        res_in->success = true;
    }

    size_t msg_len = strlen(message);
    res_in->message.data = (char *) message;
    res_in->message.size = msg_len;
    res_in->message.capacity = msg_len + 1;  // Add null terminated byte
}

// rcl_ret_t actuator_v2_dynamixel_update_status(void) {
//     riptide_msgs2__msg__DynamixelStatus status;
//     actuator_dxlitr_t itr;

//     // Iterate through dynamixels, publishing status
//     actuator_dxlitr_init(&itr);
//     while (actuator_dxlitr_next(&itr, &status)) {
//         RCRETCHECK(rcl_publish(&dynamixel_status_publisher, &status, NULL));
//     }

//     return RCL_RET_OK;
// }

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

// ========================================
// Initialization
// ========================================

// Define the number of executor handles required for this file
const size_t ros_actuators_num_executor_handles = 6;

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

    return RCL_RET_OK;
}

rcl_ret_t ros_actuators_fini(rcl_node_t *node) {
    RCSOFTCHECK(rcl_publisher_fini(&status_publisher, node));
    RCSOFTCHECK(rcl_publisher_fini(&busy_publisher, node));
    RCSOFTCHECK(rcl_service_fini(&torpedo_service, node));
    RCSOFTCHECK(rcl_service_fini(&dropper_service, node));
    RCSOFTCHECK(rcl_service_fini(&notify_reload_service, node));
    RCSOFTCHECK(rcl_service_fini(&arm_service, node));
    RCSOFTCHECK(rcl_publisher_fini(&dynamixel_status_publisher, node));
    RCSOFTCHECK(rcl_service_fini(&move_home_service, node));
    RCSOFTCHECK(rcl_service_fini(&set_home_service, node));

    return RCL_RET_OK;
}
