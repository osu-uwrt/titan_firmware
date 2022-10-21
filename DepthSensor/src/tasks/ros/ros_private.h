#ifndef ROS_PRIVATE_H
#define ROS_PRIVATE_H

#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include "basic_logger/logging.h"
#include "drivers/safety.h"
#include "tasks/ros.h"


#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "ros"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_FATAL("Failed status on in " __FILE__ ":%d : %d. Aborting.",__LINE__,(int)temp_rc); panic("Unrecoverable ROS Error");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_SOFT_FAIL);}}

static inline void nanos_to_timespec(int64_t time_nanos, struct timespec *ts) {
	ts->tv_sec = time_nanos / 1000000000;
	ts->tv_nsec = time_nanos % 1000000000;
}


void depth_publisher_init(rclc_support_t *support, rcl_node_t *node, rclc_executor_t *executor);
void depth_publisher_fini(rcl_node_t *node);

#endif