#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define DYNAMIXEL_PACKET_BUFFER_SIZE 128

// TODO: add more specific errors
enum dynamixel_error { 
    DYNAMIXEL_ERROR_NONE = 0,
    /* An internal error in the driver */
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

typedef void (*dynamixel_error_cb)(enum dynamixel_error error_code);

typedef void (*dynamixel_event_cb)(enum dynamixel_event, dynamixel_id id);

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb, dynamixel_event_cb event_cb);

bool dynamixel_set_id(dynamixel_id old, dynamixel_id new);

bool dynamixel_enable_torque(dynamixel_id id, bool enabled);

bool dynamixel_set_target_position(dynamixel_id id, uint32_t pos);

void dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom);

/* How control servo (?) */

#endif