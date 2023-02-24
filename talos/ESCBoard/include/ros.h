#ifndef ROS_H
#define ROS_H

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Aborting.",__LINE__,(int)temp_rc); return temp_rc;}}
#define RCRETCHECK_QUIET(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return temp_rc;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_WARN("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc);}}

rcl_ret_t ros_heartbeat_pulse();

rcl_ret_t ros_update_firmware_status();

void ros_rmw_init(void);

rcl_ret_t ros_init();

void ros_fini(void);

void ros_update(void);

bool is_ros_connected(void);

bool ros_ping(void);

#endif