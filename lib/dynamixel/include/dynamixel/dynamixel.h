#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define DYNAMIXEL_PACKET_BUFFER_SIZE 128

// TODO: add more specific errors
enum dynamixel_error { 
    DYNAMIXEL_ERROR_NONE = 0,
    /* An internal error: if this is reported it is a bug in the driver. */
    DYNAMIXEL_DRIVER_ERROR,
    /* An internal error in the TTL driver */
    DYNAMIXEL_TTL_ERROR,
    /* This is triggered when a request was invalid */
    DYNAMIXEL_REQUEST_ERROR,
    /* This is triggered when a packet is not properly parsed. */
    DYNAMIXEL_PACKET_ERROR,
    /* This is flagged when a */
    HARDWARE_ERROR,
    DYNAMIXEL_CMD_QUEUE_FULL_ERROR,
    /* This is triggered when the driver encounters a packet with an ID that is not known to the driver. */
    DYNAMIXEL_INVALID_ID,
};

enum dynamixel_event { 
    /* This event is fired when the driver completes an EEPROM read of the servo. */
    DYNAMIXEL_EVENT_EEPROM_READ,
     /* This event is fired when the driver completes a RAM read of the servo. */
    DYNAMIXEL_EVENT_RAM_READ,
    /* This event is fired when the driver receives a response to a PING request. */
    DYNAMIXEL_EVENT_PING,
    // TODO: PING Good, PING Failed, etc.
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
    uint32_t homing_offset;
    uint8_t temperature_limit;
    uint16_t max_voltage_limit;
    uint16_t min_voltage_limit;
    uint16_t pwm_limit;
    uint32_t velocity_limit;
    uint32_t max_position_limit;
    uint32_t min_position_limit;
    uint8_t startup_config;
};

struct dynamixel_ram { 
    bool torque_enable;
    uint8_t led;
    uint8_t status_return_level;
    uint8_t registered_instruction;
    uint8_t hardware_error_status;
    uint16_t velocity_i_gain;
    uint16_t velocity_p_gain;
    uint16_t position_d_gain;
    uint16_t position_i_gain;
    uint16_t position_p_gain;
    uint16_t feedforward_2nd_gain;
    uint16_t feedforward_1st_gain;
    uint8_t bus_watchdog;
    uint16_t goal_pwm;
    uint32_t goal_velocity;
    uint32_t profile_accel;
    uint32_t profile_vel;
    uint32_t goal_position;
    uint16_t realtime_tick;
    uint8_t moving;
    uint8_t moving_status;
    uint16_t present_pwm;
    uint16_t present_load;
    uint32_t present_velocity;
    uint32_t present_position;
    uint32_t velocity_trajectory;
    uint32_t position_trajectory;
    uint16_t present_input_voltage;
    uint16_t present_temperature;
    uint8_t backup_ready;
};

typedef uint8_t dynamixel_id;


// TODO: add cause string
typedef void (*dynamixel_error_cb)(enum dynamixel_error error_code);

typedef void (*dynamixel_event_cb)(enum dynamixel_event, dynamixel_id id);

/**
 * @brief Initializes the driver the specified list of servo IDs. 
 * 
 * @param id_list a list of IDs for the servos that will be used. 
 * @param id_cnt the number of IDs in the id_list
 * @param error_cb the error callback that will be used by the driver.
 * @param event_cb the event callback that will be used by the driver. 
*/
void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb, dynamixel_event_cb event_cb);

bool dynamixel_set_id(dynamixel_id old, dynamixel_id new);

bool dynamixel_set_target_position(dynamixel_id id, uint32_t pos);

/** 
 * @brief Send a request to read the EEPROM off of the servo.
 * 
 * Once the EEPROM is returned from the servo, it will be cached internally in the driver. You can get
 * the EEPROM settings by calling `dynamixel_get_eeprom` or by listening for a `DYNAMIXEL_EVENT_EEPROM_READ` 
 * in the event handler. 
 * 
 * Note: This does not send the request immediately. Instead, this request will be placed in the
 * drivers internal command queue and will be sent at some point in the future.
 * 
 * @param id the ID of the servo
*/
bool dynamixel_read_eeprom(dynamixel_id id);

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
bool dynamixel_enable_torque(dynamixel_id id, bool enabled);

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
void dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom);

void dynamixel_get_ram(dynamixel_id id, struct dynamixel_ram *ram);

/* How control servo (?) */

#endif