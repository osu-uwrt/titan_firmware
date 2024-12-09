#include "actuators.h"
#include "ros_internal.h"
#include "safety_interface.h"

#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

static const rclc_parameter_options_t param_server_options = { .notify_changed_over_dds = true, .max_params = 7 };

static rclc_parameter_server_t param_server;

#define IS_VALID_TIMING(num) ((num) > 0 && (num) < (1 << 16))
bool actuator_handle_parameter_change(const Parameter *param) {
    if (!actuators_initialized) {
        return false;
    }

    // All parameters are int types
    if (param->value.type != RCLC_PARAMETER_INT) {
        return false;
    }

    if (!strcmp(param->name.data, "claw_timing_ms")) {
        if (IS_VALID_TIMING(param->value.integer_value)) {
            return claw_set_timings(param->value.integer_value, param->value.integer_value);
        }
        else {
            return false;
        }
    }
    else if (!strcmp(param->name.data, "dropper_active_timing_ms")) {
        if (IS_VALID_TIMING(param->value.integer_value)) {
            return dropper_set_timings(param->value.integer_value);
        }
        else {
            return false;
        }
    }
#define ELSE_IF_TORPEDO_PARAMETER(coil_lower, coil_upper)                                                              \
    else if (!strcmp(param->name.data, "torpedo_" #coil_lower "_timing_us")) {                                         \
        if (IS_VALID_TIMING(param->value.integer_value)) {                                                             \
            return torpedo_set_timings(1, ACTUATOR_TORPEDO_TIMING_##coil_upper##_TIME, param->value.integer_value) &&  \
                   torpedo_set_timings(2, ACTUATOR_TORPEDO_TIMING_##coil_upper##_TIME, param->value.integer_value);    \
        }                                                                                                              \
        else {                                                                                                         \
            return false;                                                                                              \
        }                                                                                                              \
    }
    ELSE_IF_TORPEDO_PARAMETER(coil1_on, COIL1_ON)
    ELSE_IF_TORPEDO_PARAMETER(coil1_2_delay, COIL1_2_DELAY)
    ELSE_IF_TORPEDO_PARAMETER(coil2_on, COIL2_ON)
    ELSE_IF_TORPEDO_PARAMETER(coil2_3_delay, COIL2_3_DELAY)
    ELSE_IF_TORPEDO_PARAMETER(coil3_on, COIL3_ON)
    else {
        return false;
    }
}

static bool on_parameter_changed(__unused const Parameter *old_param, const Parameter *new_param,
                                 __unused void *context) {
    if (new_param == NULL) {
        return false;
    }

    if (actuator_handle_parameter_change(new_param)) {
        // Nothing to be done on successful parameter change
        return true;
    }
    else {
        LOG_WARN("Unexpected parameter %s with type %d changed", new_param->name.data, new_param->value.type);
        safety_raise_fault(FAULT_ROS_ERROR);
        return false;
    }
}

const size_t actuator_v1_parameters_num_executor_handles = RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;

rcl_ret_t actuator_v1_parameters_init(rcl_node_t *node, rclc_executor_t *executor) {
    RCRETCHECK(rclc_parameter_server_init_with_option(&param_server, node, &param_server_options));
    RCRETCHECK(rclc_executor_add_parameter_server(executor, &param_server, on_parameter_changed));

    // Add all of our paramters
    RCRETCHECK(rclc_add_parameter(&param_server, "claw_timing_ms", RCLC_PARAMETER_INT));
    RCRETCHECK(rclc_add_parameter(&param_server, "dropper_active_timing_ms", RCLC_PARAMETER_INT));

    RCRETCHECK(rclc_add_parameter(&param_server, "torpedo_coil1_on_timing_us", RCLC_PARAMETER_INT));
    RCRETCHECK(rclc_add_parameter(&param_server, "torpedo_coil1_2_delay_timing_us", RCLC_PARAMETER_INT));
    RCRETCHECK(rclc_add_parameter(&param_server, "torpedo_coil2_on_timing_us", RCLC_PARAMETER_INT));
    RCRETCHECK(rclc_add_parameter(&param_server, "torpedo_coil2_3_delay_timing_us", RCLC_PARAMETER_INT));
    RCRETCHECK(rclc_add_parameter(&param_server, "torpedo_coil3_on_timing_us", RCLC_PARAMETER_INT));

    RCRETCHECK(rclc_parameter_set_int(&param_server, "claw_timing_ms", 4500));
    RCRETCHECK(rclc_parameter_set_int(&param_server, "dropper_active_timing_ms", 250));

    RCRETCHECK(rclc_parameter_set_int(&param_server, "torpedo_coil1_on_timing_us", 23000));
    RCRETCHECK(rclc_parameter_set_int(&param_server, "torpedo_coil1_2_delay_timing_us", 250));
    RCRETCHECK(rclc_parameter_set_int(&param_server, "torpedo_coil2_on_timing_us", 15000));
    RCRETCHECK(rclc_parameter_set_int(&param_server, "torpedo_coil2_3_delay_timing_us", 250));
    RCRETCHECK(rclc_parameter_set_int(&param_server, "torpedo_coil3_on_timing_us", 13000));

    return RCL_RET_OK;
}

void actuator_v1_parameters_fini(rcl_node_t *node) {
    RCSOFTCHECK(rclc_parameter_server_fini(&param_server, node));
}
