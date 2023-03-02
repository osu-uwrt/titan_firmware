#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

// ========================================
// Configuration
// ========================================

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DSHOT, Enable/disable assertions in the dshot module, type=bool, default=0, group=ESCBoard
#ifndef PARAM_ASSERTIONS_ENABLED_DSHOT
#define PARAM_ASSERTIONS_ENABLED_DSHOT 0
#endif

// The number of thrusters controlled by the board - Max value of 4
#define NUM_THRUSTERS 8

// The rate that dshot transmitted to the ESCs in microseconds
#define DSHOT_TX_RATE_US 1500

// The minimum rate that dshot_update_thrusters must be called before disabling thrusters in milliseconds
#define DSHOT_MIN_UPDATE_RATE_MS 200

// The amount of time in milliseconds thrusters will be disabled for if update_thrusters isn't called within the update rate timeout
#define DSHOT_UPDATE_DISABLE_TIME_MS 1000

// The delay in milliseconds that thrusters will be disabled for on ESC power-up
#define DSHOT_WAKEUP_DELAY_MS 5000

// The dshot rate in KHz, must be a valid dshot rate (DSHOT150, DSHOT300, DSHOT600, DSHOT1200)
#define DSHOT_RATE 300

// Number of packets that can be missed before the telemetry packet is marked as invalid
#define TELEM_MAX_MISSED_PACKETS 3

// The hardware alarm to use for periodic transfers
#define dshot_hardware_alarm_num 1

// ========================================
// Data Types
// ========================================

struct dshot_rpm_telemetry {
    int missed_count;       // Missed packet counter (incremented every TX attempt, zeroed after successful rx)
    uint16_t rpm_period_us; // If the RPM period is valid
    bool valid;             // If the packet is valid
};


// ========================================
// Exported Variables - Do not write
// ========================================

/**
 * @brief Boolean if dshot_init has been called
 */
extern bool dshot_initialized;

/**
 * @brief Contains RPM telemetry received from the ESCs via bidirectional dshot for each thruster in NUM_THRUSTERS
 * If the valid bool is false, then the other fields are considered invalid
 */
extern struct dshot_rpm_telemetry dshot_rpm_data[];

/**
 * @brief Contains boolean for each thruster in NUM_THRUSTERS reporting if the rpm period is for the motor moving backwards.
 */
extern bool dshot_rpm_reversed[];


// ========================================
// Exported Methods
// ========================================

/**
 * @brief Immediately sends stop command to all thrusters.
 * If esc pwm has not been initialized yet, this call does nothing
 */
void dshot_stop_thrusters(void);

/**
 * @brief Updates thruster values to the specified values.
 * This will convert the value range -999 to 999 to the proper dshot command
 *
 * @attention DShot must be initialized before calling this function
 * @attention Cannot be called in interrupt context
 *
 * @param throttle_values A NUM_THRUSTERS long array of throttle values in range (-999 to 999) [one for each thruster]
 */
void dshot_update_thrusters(const int16_t *throttle_values);

/**
 * @brief Notify that the physical kill switch has changed state
 *
 * @param asserting_kill True if the kill switch is asserting kill
 */
void dshot_notify_physical_kill_switch_change(bool asserting_kill);

/**
 * @brief Initialize dshot and starts outputting thruster neutral commands
 */
void dshot_init(void);


#endif  // DSHOT_H