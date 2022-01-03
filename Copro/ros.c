#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"

#include "ros.h"
#include "safety.h"
#include "lux_sensor.h"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float64 send_msg;
std_msgs__msg__Int32 recv_msg;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Aborting.\n",__LINE__,(int)temp_rc); panic("Unrecoverable ROS Error");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Continuing.\n",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_SOFT_FAIL);}}


// ========================================
// ROS Callbacks
// ========================================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		if (lux_initialized)
			send_msg.data = lux_read();
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		//printf("Sent: %d\n", send_msg.data);
		send_msg.data++;
	}
}

// Implementation example:
void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n", msg->data);
	if (msg->data == 3) {
		*(uint32_t*)(0xFFFFFFFC) = 0xDEADBEEF;
	} else if (msg->data == 4) {
		void (*bad_jump)(void) = (void*)(0xFFFFFFF0);
		bad_jump();
	} else if (msg->data == 5){
		panic("IT DO GO DOWN!");
	}
}

// ========================================
// Public Methods
// ========================================

void ros_wait_for_connection(void) {
	// Make sure this is less than the watchdog timeout
    const int timeout_ms = 1000; 

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);
}

void ros_start(const char* namespace) {
    allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "coprocessor_node", namespace, &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
		"pico_publisher"));

  	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"pico_subscriber"));

	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	send_msg.data = 0;
}

void ros_spin_ms(long ms) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms));
}

void ros_cleanup(void) {
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
}