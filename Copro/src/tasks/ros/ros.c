#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "basic_logger/logging.h"
#include "pico_uart_transports.h"

#include "ros_private.h"


void ros_wait_for_connection(void) {
	// Make sure this is less than the watchdog timeout
    const int timeout_ms = 1000;

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);
}

static rcl_allocator_t allocator = {0};
static rclc_support_t support = {0};
static rcl_node_t node = {0};
static rclc_executor_t executor = {0};

void ros_start(const char* namespace) {
    allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "coprocessor_node", namespace, &support));

	// create executor
	const uint num_executor_tasks = 7 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, num_executor_tasks, &allocator));

	parameter_server_init(&node, &executor);
	depth_publisher_init(&support, &node, &executor);
	state_publish_init(&support, &node, &executor);
	subscriptions_init(&node, &executor);
}

void ros_spin_ms(long ms) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms)));
}

void ros_cleanup(void) {
	parameter_server_fini(&node);
	depth_publisher_fini(&node);
	state_publish_fini(&node);
	subscriptions_fini(&node);

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
}