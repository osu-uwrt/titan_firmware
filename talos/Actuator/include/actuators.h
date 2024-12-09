#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <riptide_msgs2/msg/actuator_status.h>

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ACTUATORS, Enable/disable assertions for the actuators module, type=bool, default=0, group=talos/actuators
#ifndef PARAM_ASSERTIONS_ENABLED_ACTUATORS
#define PARAM_ASSERTIONS_ENABLED_ACTUATORS 0
#endif

// ========================================
// Actuator Global
// ========================================

/**
 * @brief Boolean for if actuators have been initialized
 */
extern bool actuators_initialized;

/**
 * @brief Actuators arm state, true if armed
 */
extern volatile bool actuators_armed;

/**
 * @brief Initializes actuators
 */
void actuators_initialize(void);

/**
 * @brief Arms actuator system
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool true on success
 */
bool actuators_arm(const char **errMsgOut);

/**
 * @brief Disarms actuator system.
 * This aborts all active operations in process and brings actuators to a safe state
 * @note This is safe to call in interrupts (Note for implementer, make sure it is safe to call in interrupts)
 */
void actuators_disarm(void);

/**
 * @brief Report is the actuators are currently busy performing an action.
 * This is true if any of the actuators are performing a request which should finish within a bounded amount of time.
 * This is a useful to wait to clear before performing any additional actuator actions.
 *
 * @return true Actuators are busy performing command
 * @return false Actuators are not busy (however they could be in an error state)
 */
bool actuators_get_busy(void);

// ========================================
// Claw
// ========================================

/**
 * @brief Returns the current state of the claw
 *
 * @return uint8_t Claw's Current State following ActuatorStatus message
 */
uint8_t claw_get_state(void);

/**
 * @brief Attempts to begin opening the claw
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool true when opened or running, false otherwise.
 */
bool claw_open(const char **errMsgOut);

/**
 * @brief Attempts to begin closing the claw
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool true when opened or running, false otherwise.
 */
bool claw_close(const char **errMsgOut);

// ========================================
// Droppers
// ========================================

/**
 * @brief Returns the current state of the marker dropper
 *
 * @return uint8_t Dropper's Current State following ActuatorStatus message
 */
uint8_t dropper_get_state(void);

/**
 * @brief Returns the number of droppers available
 *
 * @return uint8_t
 */
uint8_t dropper_get_available(void);

/**
 * @brief Attempts to drop the requested marker
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool dropper_drop_marker(const char **errMsgOut);

/**
 * @brief Notifies the dropper system that the droppers have been reloaded
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool dropper_notify_reload(const char **errMsgOut);

// ========================================
// Torpedos
// ========================================

/**
 * @brief Returns the current state of the torpedo system
 *
 * @return uint8_t Torpedo System's Current State following ActuatorStatus message
 */
uint8_t torpedo_get_state(void);

/**
 * @brief Returns the number of torpedos available
 *
 * @return uint8_t
 */
uint8_t torpedo_get_available(void);

/**
 * @brief Attempts to fire the requested torpedo
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return if the torpedo successfully fired
 */
bool torpedo_fire(const char **errMsgOut);

/**
 * @brief Notifies the dropper system that the torpedos have been reloaded
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool torpedo_notify_reload(const char **errMsgOut);

// ========================================
// Actuator Configuration
// ========================================

#ifdef ACTUATOR_V1_SUPPORT
// Only mark 1 actuators have timing configuration

enum torpedo_timing_type {
    ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME = 0,
    ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME = 1,
    ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME = 2,
    ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME = 3,
    ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME = 4,

    ACTUATOR_NUM_TORPEDO_TIMINGS
};

/**
 * @brief Attempts to set the claw timings
 *
 * @param timings Message struct containing claw timings
 * @return bool Result on setting the claw timings (true on success)
 */
bool claw_set_timings(uint16_t open_time_ms, uint16_t close_time_ms);

/**
 * @brief Attempts to set the marker dropper timings
 *
 * @param uint16_t active_time_ms dropper timings
 * @return bool Result of setting the dropper timings
 */
bool dropper_set_timings(uint16_t active_time_ms);

/**
 * @brief Attempts to set the torpedo timings
 *
 * @param timings Message struct containing torpedo timings
 * @return bool Result on setting the torpedo timings (true on success)
 */
bool torpedo_set_timings(uint8_t torpedo_num, enum torpedo_timing_type timing_type, uint16_t time_us);

#endif

#if ACTUATOR_V2_SUPPORT

#include <riptide_msgs2/msg/dynamixel_status.h>

#include <stddef.h>

typedef size_t actuator_dxlitr_t;

/**
 * @brief Initialize dynamixel status iterator object
 *
 * @param itr Pointer to iterator object to initialize
 */
void actuator_dxlitr_init(actuator_dxlitr_t *itr);

/**
 * @brief Retrieves the status message for the current dynamixel organized by the iterator
 *
 * @param itr Pointer to iterator object, updated by this call
 * @param status_out Pointer to store the current dynamixel status
 * @return true Status was successfully retrieved and the iterator was advanced
 * @return false End of iterator reached
 */
bool actuator_dxlitr_next(actuator_dxlitr_t *itr, riptide_msgs2__msg__DynamixelStatus *status_out);

/**
 * @brief Attempts to set the current torpedo marker servo position as its home position.
 * This is useful when bringing up new torpedo marker systems as this allows each torpedo marker to have its own
 * absolute position which is known as the "home" position.
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool torpedo_marker_set_home(const char **errMsgOut);

/**
 * @brief Attempts to move the current torpedo marker servo position to its home position.
 * This is useful for moving the marker dropper back to its home position if stuck due to an unexpected disconnect
 * or movement when powered off (as fire/drop commands do not work if the servo is not in its home position).
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool torpedo_marker_move_home(const char **errMsgOut);

/**
 * @brief Attempts to set the current claw servo position as its closed position.
 * This is useful if the claw ever loses its continuous rotation offset and will re-zero it.
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @return bool Result for the attempted command (true on success)
 */
bool claw_set_closed_position(const char **errMsgOut);

/**
 * @brief Attempts to move the claw by a given delta. This is useful for moving the claw to a good close position if
 * the home position is ever lost.
 *
 * @param errMsgOut Optional return for error message why the request failed
 * @param move_delta The number of ticks to add/subtract from the current dynamixel position
 * @return bool Result for the attempted command (true on success)
 */
bool claw_creep_delta(const char **errMsgOut, int32_t move_delta);

#endif

#endif
