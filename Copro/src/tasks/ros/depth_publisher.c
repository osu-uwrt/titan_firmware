#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include <riptide_msgs2/msg/depth.h>

#include "basic_logger/logging.h"

#include "drivers/safety.h"
#include "hw/depth_sensor.h"

#include "ros_private.h"


static rcl_publisher_t depth_publisher;
static riptide_msgs2__msg__Depth depth_msg;
static rcl_timer_t depth_publisher_timer;
static char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
static const float depth_variance = 0.003;  					// TODO: Load from config file or something
static const int depth_publish_rate_ms = 50;

static void depth_publisher_timer_callback(rcl_timer_t * timer, __unused int64_t last_call_time) {
	if (timer != NULL && depth_reading_valid()) {

		struct timespec ts;
		nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
		depth_msg.header.stamp.sec = ts.tv_sec;
		depth_msg.header.stamp.nanosec = ts.tv_nsec;

		depth_msg.depth = -depth_read();
		RCSOFTCHECK(rcl_publish(&depth_publisher, &depth_msg, NULL));
	}
}

void depth_publisher_init(rclc_support_t *support, rcl_node_t *node, rclc_executor_t *executor) {
	RCCHECK(rclc_publisher_init(
		&depth_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth),
		"depth/raw",
		&rmw_qos_profile_sensor_data));

	RCCHECK(rclc_timer_init_default(
		&depth_publisher_timer,
		support,
		RCL_MS_TO_NS(depth_publish_rate_ms),
		depth_publisher_timer_callback));

	RCCHECK(rclc_executor_add_timer(executor, &depth_publisher_timer));

	depth_msg.header.frame_id.data = depth_frame;
	depth_msg.header.frame_id.capacity = sizeof(depth_frame);
	depth_msg.header.frame_id.size = strlen(depth_frame);
	depth_msg.variance = depth_variance;
}

void depth_publisher_fini(rcl_node_t *node) {
	RCCHECK(rcl_publisher_fini(&depth_publisher, node));
	RCCHECK(rcl_timer_fini(&depth_publisher_timer));
}