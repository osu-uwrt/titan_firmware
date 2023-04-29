#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ACTUATORS, Enable/disable assertions for the actuators module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_ACTUATORS
#define PARAM_ASSERTIONS_ENABLED_ACTUATORS 0
#endif

// TODO: Update state to follow the current system better

enum claw_state {
    CLAW_STATE_UNINITIALIZED = 0,
    CLAW_STATE_DISARMED,
    CLAW_STATE_UNKNOWN_POSITION,
    CLAW_STATE_OPENED,
    CLAW_STATE_CLOSED,
    CLAW_STATE_OPENING,
    CLAW_STATE_CLOSING,
    CLAW_STATE_ERROR
};

enum dropper_state {
    DROPPER_STATE_UNINITIALIZED = 0,
    DROPPER_STATE_DISARMED,
    DROPPER_STATE_READY,
    DROPPER_STATE_DROPPING,
    DROPPER_STATE_ERROR
};

enum torpedo_state {
    TORPEDO_STATE_UNINITIALIZED = 0,
    TORPEDO_STATE_DISARMED,
    TORPEDO_STATE_CHARGING,
    TORPEDO_STATE_READY,
    TORPEDO_STATE_FIRING,
    TORPEDO_STATE_ERROR
};

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
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return bool true on success
 */
bool actuators_arm(const char **errMsgOut);

/**
 * @brief Disarms actuator system.
 * This aborts all active operations in process and brings actuators to a safe state
 * @note This is safe to call in interrupts (Note for implementer, make sure it is safe to call in interrupts)
 */
void actuators_disarm(void);


// ========================================
// Claw
// ========================================

/**
 * @brief Returns the current state of the claw
 *
 * @return enum claw_state Claw's Current State
 */
enum claw_state claw_get_state(void);

/**
 * @brief Attempts to begin opening the claw
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return bool true when opened or running, false otherwise.
 */
bool claw_open(const char **errMsgOut);

/**
 * @brief Attempts to begin closing the claw
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return bool true when opened or running, false otherwise.
 */
bool claw_close(const char **errMsgOut);


// ========================================
// Droppers
// ========================================

/**
 * @brief Returns the current state of the marker dropper
 *
 * @param dropper_id: The dropper to get the state of (id starts at 1)
 * @return enum dropper_state dropper's Current State
 */
enum dropper_state dropper_get_state(void);

/**
 * @brief Attempts to drop the requested marker
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return bool Result for the attempted command (true on success)
 */
bool dropper_drop_marker(const char **errMsgOut);

/**
 * @brief Notifies the dropper system that the droppers have been reloaded
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return bool Result for the attempted command (true on success)
 */
bool dropper_notify_reload(const char **errMsgOut);


// ========================================
// Torpedos
// ========================================

/**
 * @brief Returns the current state of the torpedo system
 *
 * @return enum torpedo_state Torpedo System's Current State
 */
enum torpedo_state torpedo_get_state(void);

/**
 * @brief Attempts to fire the requested torpedo
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
 * @return if the torpedo successfully fired
 */
bool torpedo_fire(const char **errMsgOut);

/**
 * @brief Notifies the dropper system that the torpedos have been reloaded
 *
 * @param errMsgOut Optional return for error message why the request failed. Can be NULL
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

#endif