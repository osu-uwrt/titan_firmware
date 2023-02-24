#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include <std_msgs/msg/empty.h>
#include <riptide_msgs2/msg/depth.h>

#include "basic_logger/logging.h"

#include "hw/depth_sensor.h"
#include "ros/ros.h"

static rcl_publisher_t depth_publisher;
static riptide_msgs2__msg__Depth depth_msg;
static char depth_frame[] = ROBOT_NAMESPACE "/pressure_link";
static const float depth_variance = 0.003;  					// TODO: Load from config file or something

static inline void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
	ts->tv_sec = time_nanos / 1000000000;
	ts->tv_nsec = time_nanos % 1000000000;
}

rcl_ret_t ros_update_depth_publisher() {
    if (depth_reading_valid()) {
		struct timespec ts;
		nanos_to_timespec(rmw_uros_epoch_nanos(), &ts);
		depth_msg.header.stamp.sec = ts.tv_sec;
		depth_msg.header.stamp.nanosec = ts.tv_nsec;

		depth_msg.depth = -depth_read();
		RCRETCHECK(rcl_publish(&depth_publisher, &depth_msg, NULL));
	}

    return RCL_RET_OK;
}

rcl_ret_t ros_depth_publisher_init(rcl_node_t *node) { 
	RCRETCHECK(rclc_publisher_init(
		&depth_publisher,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, Depth),
		"depth/raw",
		&rmw_qos_profile_sensor_data));

	depth_msg.header.frame_id.data = depth_frame;
	depth_msg.header.frame_id.capacity = sizeof(depth_frame);
	depth_msg.header.frame_id.size = strlen(depth_frame);
	depth_msg.variance = depth_variance;

	return RCL_RET_OK;
}

void ros_depth_publisher_fini(rcl_node_t *node) { 
	RCSOFTCHECK(rcl_publisher_fini(&depth_publisher, node));
}