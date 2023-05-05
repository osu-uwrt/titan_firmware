#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "hardware/pio.h"

enum dynamixel_error_code {
    DYNAMIXEL_ERROR_NONE = 0,
    /* An internal error: if this is reported it is a bug in the driver. */
    DYNAMIXEL_DRIVER_ERROR,
    /* An internal error in the TTL driver */
    DYNAMIXEL_TTL_ERROR,
    /* This is triggered when a request was invalid */
    DYNAMIXEL_REQUEST_ERROR,
    /* This is triggered when a packet is not properly parsed. */
    DYNAMIXEL_PACKET_ERROR,
    /* This is triggered when the dynamixel reports it is in an error state (error field set) */
    DYNAMIXEL_HARDWARE_ERROR,
    /* The command queue is full and the most recently queued command was dropped */
    DYNAMIXEL_CMD_QUEUE_FULL_ERROR,
    /* This is triggered when the driver encounters a packet with an ID that is not known to the driver. */
    DYNAMIXEL_INVALID_ID,
};

enum dynamixel_error_source {
    DYNAMIXEL_SOURCE_SCHEDULE = 0,
    DYNAMIXEL_SOURCE_COMMS = 1
};

typedef union dynamixel_error {
    struct {
        enum dynamixel_error_code error:8;
        enum dynamixel_error_source error_source:1;
        uint16_t line:15;
        uint8_t wrapped_error_code;
    } __packed fields;
    uint32_t data;
} dynamixel_error_t;
static_assert(sizeof(dynamixel_error_t) == sizeof(uint32_t), "Struct did not pack properly");

enum dynamixel_event {
    /**
     * @brief Reports that a servo has connected.
     * This event fires after the EEPROM and RAM has been read off the device
     */
    DYNAMIXEL_EVENT_CONNECTED,
    /**
     * @brief Reports that a servo has disconnected.
     * This is also sent if the if the first attempt to connect to the servo fails
     */
    DYNAMIXEL_EVENT_DISCONNECTED,
     /**
      * @brief Reports periodic RAM read of the servo.
      */
    DYNAMIXEL_EVENT_RAM_READ,
    /**
     * @brief Reports that the alert bit has been set on the servo
     *
     * Note that the ram should be valid at this point to read hardware_error_status
     */
    DYNAMIXEL_EVENT_ALERT,
};

// TODO: convert some of these to enums
struct dynamixel_eeprom {
    uint16_t model_num;
    uint32_t model_info;
    uint8_t firmware_version;
    uint8_t id;
    uint8_t baud_rate;
    uint8_t return_delay_time;
    uint8_t drive_mode;
    uint8_t operating_mode;
    uint8_t secondary_id;
    uint8_t protocol_type;
    int32_t homing_offset;
    uint32_t moving_threshold;
    uint8_t temperature_limit;
    uint16_t max_voltage_limit;
    uint16_t min_voltage_limit;
    uint16_t pwm_limit;
    uint32_t velocity_limit;
    uint32_t max_position_limit;
    uint32_t min_position_limit;
    uint8_t startup_config;
    uint8_t shutdown;
};

struct dynamixel_ram {
    bool torque_enable;
    bool led;
    uint8_t status_return_level;
    bool registered_instruction;
    uint8_t hardware_error_status;
    uint16_t velocity_i_gain;
    uint16_t velocity_p_gain;
    uint16_t position_d_gain;
    uint16_t position_i_gain;
    uint16_t position_p_gain;
    uint16_t feedforward_2nd_gain;
    uint16_t feedforward_1st_gain;
    int8_t bus_watchdog;
    int16_t goal_pwm;
    int32_t goal_velocity;
    uint32_t profile_accel;
    uint32_t profile_vel;
    int32_t goal_position;
    uint16_t realtime_tick;
    bool moving;
    uint8_t moving_status;
    int16_t present_pwm;
    int16_t present_load;
    int32_t present_velocity;
    int32_t present_position;
    uint32_t velocity_trajectory;
    uint32_t position_trajectory;
    uint16_t present_input_voltage;
    uint8_t present_temperature;
    bool backup_ready;
};

typedef uint8_t dynamixel_id;


// TODO: add cause string
typedef void (*dynamixel_error_cb)(dynamixel_error_t error);

typedef void (*dynamixel_event_cb)(enum dynamixel_event, dynamixel_id id);

/**
 * @brief Initializes the driver the specified list of servo IDs.
 *
 * @param pio The PIO block to use
 * @param sm The PIO state machine to use
 * @param pin The pin with the dynammixel servo
 * @param id_list Array of servos to monitor
 * @param id_cnt Count of servos to monitor
 * @param error_cb Callback to report an error (events which cannot be gracefully handled by the dynamixel driver)
 * @param event_cb Callback to report any scheduler events
*/
void dynamixel_init(PIO pio, uint sm, uint pin,
                    const dynamixel_id *id_list, size_t id_cnt,
                    dynamixel_error_cb error_cb,
                    dynamixel_event_cb event_cb);

void dynamixel_set_id(dynamixel_id old, dynamixel_id new);

void dynamixel_set_target_position(dynamixel_id id, int32_t pos);

/**
 * @brief Sets the torque enabled flag in the dynamixel's RAM.
 *
 * When torque is enabled, the servo will attempt to move to it's goal position or goal velocity.
 * Additionally, the EEPROM will be locked and all writes to the EEPROM will fail.
 *
 * Note: This does not send the packet immediately. Instead, this request will be placed in the
 * drivers internal command queue and will be sent at some point in the future.
 *
 * @param id the ID of the servo
 * @param enabled what to set the enable torque flag to.
*/
void dynamixel_enable_torque(dynamixel_id id, bool enabled);

/**
 * @brief Gets a copy of the eeprom last read from the servo.
 *
 * Note: This does not actually request the EEPROM from the servo. Instead this will return
 * the settings from the last time the EEPROM was read. To actually request the EEPROM be read,
 * use `dynamixel_read_eeprom`.
 *
 * @param id specifies which servo eeprom to get. This ID must be passed to the driver at dynamixel_init.
 * @param eeprom the struct to copy the EEPROM variables to.
*/
bool dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom);

/**
 * @brief Gets a copy of the ram last reD from the servo.
 *
 * @param id ID of the dynamixel read
 * @param ram Pointer to location to store RAM
 * @return true The ram was successfully read
 * @return false The ram was not read or is stale
 */
bool dynamixel_get_ram(dynamixel_id id, struct dynamixel_ram *ram);

/**
 * @brief Returns if the corresponding dynamixel is connected.
 * If this returns true, RAM data is considered valid
 *
 * @param id The ID to lookup
 * @return true If the dynamixel is connected
 * @return false If the dynamixel is not connected
 */
bool dynamixel_check_connected(dynamixel_id id);

/**
 * @brief Gets the current position of the servo
 *
 * @param id The servo to lookup
 * @param position_out The position
 * @return true Position successfully read
 * @return false Position failed to read
 */
bool dynamixel_get_position(dynamixel_id id, int32_t *position_out);

/* How control servo (?) */

#endif