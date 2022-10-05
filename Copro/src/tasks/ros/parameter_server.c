#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include "basic_logger/logging.h"

#include "drivers/safety.h"
#include "hw/actuator.h"

#include "ros_private.h"


static const rclc_parameter_options_t param_server_options = {
      .notify_changed_over_dds = true,
      .max_params = 13 };

static rclc_parameter_server_t param_server;

static bool on_parameter_changed(__unused const Parameter * old_param, const Parameter * new_param, __unused void * context)
{
	if (new_param == NULL) {
		return false;
	}

	if (actuator_handle_parameter_change(new_param)) {
		// Nothing to be done on successful parameter change
		return true;
	} else {
		LOG_WARN("Unexpected parameter %s with type %d changed", new_param->name.data, new_param->value.type);
		safety_raise_fault(FAULT_ROS_SOFT_FAIL);
		return false;
	}
}

void parameter_server_init(rcl_node_t *node, rclc_executor_t *executor) {
  	RCCHECK(rclc_parameter_server_init_with_option(&param_server, node, &param_server_options));
	RCCHECK(rclc_executor_add_parameter_server(executor, &param_server, on_parameter_changed));

	RCCHECK(actuator_create_parameters(&param_server));
	// TODO: Add cooling threshold as a parameter
}

void parameter_server_fini(rcl_node_t *node){
	rclc_parameter_server_fini(&param_server, node);
}