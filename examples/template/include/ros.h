#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Aborting.\n",__LINE__,(int)temp_rc); return temp_rc;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_ret_t ros_heartbeat_pulse();

void ros_rmw_init(void);

bool ros_init();

void ros_fini(void);

void ros_update(void);

bool is_ros_connected(void);

bool ros_ping(void);