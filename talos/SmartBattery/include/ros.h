#ifndef ROS_H
#define ROS_H

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "safety_interface.h"
#include "bq40z80.h"

#define RCRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_ERROR("Failed status on in " __FILE__ ":%d : %d. Aborting.",__LINE__,(int)temp_rc); safety_raise_fault(FAULT_ROS_ERROR); return temp_rc;}}
#define RCSOFTRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return temp_rc;}}
#define RCSOFTRETVCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){LOG_WARN("Failed status on in " __FILE__ ":%d : %d. Continuing.",__LINE__,(int)temp_rc);}}

// ========================================
// ROS Core Functions
// ========================================

/**
 * @brief Set up error handling handler to report any RMW errors
 * Makes life a *lot* easier, as long as debug logging is enabled
 */
void ros_rmw_init_error_handling(void);

/**
 * @brief Attempt to initialize ROS after a successful ping from the agent
 *
 * @param board_id The ID for the board
 * @return rcl_ret_t Return error code
 */
rcl_ret_t ros_init(uint8_t board_id);

/**
 * @brief Clean up a previously initialized or attempted initialized ROS connection
 *
 * @attention Ensure this is ALWAYS called after `ros_init` is called, whether or not it succeeds,
 * to avoid memory leaks.
 */
void ros_fini(void);

/**
 * @brief Spin the executor once and handle any incoming packets
 */
void ros_spin_executor(void);

/**
 * @brief Reports if ROS is connected, calculated based on if a heartbeat message successfully sends
 *
 * @return true ROS is still connected
 * @return false Enough heartbeats have failed that the ROS connection is considered dead
 */
bool is_ros_connected(void);

/**
 * @brief Attempt to ping the agent
 *
 * @return true Ping successful
 * @return false No response received from the agent
 */
bool ros_ping(void);

/**
 * @brief Determine if a power cycle has been requested
 *
 * if this function returns true, it will clear the internal flag and determine the request as serviced
 *
 * @return true if power cycle event has been requested
*/
bool power_cycle_requested(void);

// ========================================
// ROS Task Functions
// ========================================

rcl_ret_t ros_heartbeat_pulse(uint8_t client_id);

rcl_ret_t ros_update_firmware_status(uint8_t client_id);

rcl_ret_t ros_update_battery_status(bq_pack_info_t bq_pack_info);

// TODO: Add in any additional ROS tasks here

#endif