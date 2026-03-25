#ifndef HIWONDER_DRIVER_H
#define HIWONDER_DRIVER_H

#include "ros.h"

#include "pico/stdlib.h"

// Taken from Hiwonder datasheet
#define SERVO_MOVE_TIME_WRITE_CMD 1
#define SERVO_MOVE_TIME_READ_CMD 2
#define SERVO_MOVE_TIME_WAIT_WRITE_CMD 7
#define SERVO_MOVE_TIME_WAIT_READ_CMD 8
#define SERVO_MOVE_START_CMD 11
#define SERVO_MOVE_STOP_CMD 12
#define SERVO_ID_WRITE_CMD 13
#define SERVO_ID_READ_CMD 14
#define SERVO_ANGLE_OFFSET_ADJUST_CMD 17
#define SERVO_ANGLE_OFFSET_WRITE_CMD 18
#define SERVO_ANGLE_OFFSET_READ_CMD 19
#define SERVO_ANGLE_LIMIT_WRITE_CMD 20
#define SERVO_ANGLE_LIMIT_READ_CMD 21
#define SERVO_VIN_LIMIT_WRITE_CMD 22
#define SERVO_VIN_LIMIT_READ_CMD 23
#define SERVO_TEMP_MAX_LIMIT_WRITE_CMD 24
#define SERVO_TEMP_MAX_LIMIT_READ_CMD 25
#define SERVO_TEMP_READ_CMD 26
#define SERVO_VIN_READ_CMD 27
#define SERVO_POS_READ_CMD 28
#define SERVO_OR_MOTOR_MODE_WRITE_CMD 29
#define SERVO_OR_MOTOR_MODE_READ_CMD 30
#define SERVO_LOAD_OR_UNLOAD_WRITE_CMD 31
#define SERVO_LOAD_OR_UNLOAD_READ_CMD 32
#define SERVO_LED_CTRL_WRITE_CMD 33
#define SERVO_LED_CTRL_READ_CMD 34
#define SERVO_LED_ERROR_WRITE_CMD 35
#define SERVO_LED_ERROR_READ_CMD 36

#define SERVO_MOVE_TIME_WRITE_LEN 7
#define SERVO_MOVE_TIME_READ_LEN 3
#define SERVO_MOVE_TIME_WAIT_WRITE_LEN 7
#define SERVO_MOVE_TIME_WAIT_READ_LEN 3
#define SERVO_MOVE_START_LEN 3
#define SERVO_MOVE_STOP_LEN 3
#define SERVO_ID_WRITE_LEN 4
#define SERVO_ID_READ_LEN 3
#define SERVO_ANGLE_OFFSET_ADJUST_LEN 4
#define SERVO_ANGLE_OFFSET_WRITE_LEN 3
#define SERVO_ANGLE_OFFSET_READ_LEN 3
#define SERVO_ANGLE_LIMIT_WRITE_LEN 7
#define SERVO_ANGLE_LIMIT_READ_LEN 3
#define SERVO_VIN_LIMIT_WRITE_LEN 7
#define SERVO_VIN_LIMIT_READ_LEN 3
#define SERVO_TEMP_MAX_LIMIT_WRITE_LEN 4
#define SERVO_TEMP_MAX_LIMIT_READ_LEN 3
#define SERVO_TEMP_READ_LEN 3
#define SERVO_VIN_READ_LEN 3
#define SERVO_POS_READ_LEN 3
#define SERVO_OR_MOTOR_MODE_WRITE_LEN 7
#define SERVO_OR_MOTOR_MODE_READ_LEN 3
#define SERVO_LOAD_OR_UNLOAD_WRITE_LEN 4
#define SERVO_LOAD_OR_UNLOAD_READ_LEN 3
#define SERVO_LED_CTRL_WRITE_LEN 4
#define SERVO_LED_CTRL_READ_LEN 3
#define SERVO_LED_ERROR_WRITE_LEN 4
#define SERVO_LED_ERROR_READ_LEN 3

#define SERVO_MOVE_TIME_READ_RESPONSE_LEN 7
#define SERVO_MOVE_TIME_WAIT_READ_RESPONSE_LEN 7
#define SERVO_ID_READ_RESPONSE_LEN 4
#define SERVO_ANGLE_OFFSET_READ_RESPONSE_LEN 4
#define SERVO_ANGLE_LIMIT_READ_RESPONSE_LEN 7
#define SERVO_VIN_LIMIT_READ_RESPONSE_LEN 7
#define SERVO_TEMP_MAX_LIMIT_READ_RESPONSE_LEN 4
#define SERVO_TEMP_READ_RESPONSE_LEN 4
#define SERVO_VIN_READ_RESPONSE_LEN 5
#define SERVO_POS_READ_RESPONSE_LEN 5
#define SERVO_OR_MOTOR_MODE_READ_RESPONSE_LEN 7
#define SERVO_LOAD_OR_UNLOAD_READ_RESPONSE_LEN 4
#define SERVO_LED_CTRL_READ_RESPONSE_LEN 4
#define SERVO_LED_ERROR_READ_RESPONSE_LEN 4

#define SERVO_ID_MIN 1
#define SERVO_ID_MAX 252

#define UART_PIN 20
#define UART_BAUD 115200u
#define UART_TIMEOUT_MS 50

#define PARAMETER_MTU 4
#define HEADER_SIZE 2
#define HEADER_CODE 0x55
#define CHECKSUM_SIZE 1

#define MAX_PACKET_SIZE 10

enum servo_read_err {
    SERVO_READ_OK = 0,
    SERVO_INTERNAL_UART_ERROR,
    SERVO_BAD_RESPONSE_TYPE,
    SERVO_BAD_RESPONSE_LENGTH,
    SERVO_INCORRECT_RESPONDER,
    SERVO_BAD_CHECKSUM
};

struct ServoPacket;

typedef void (*servo_read_cb)(struct ServoPacket rx_packet, enum servo_read_err err);

typedef struct servo {
    uint8_t id;
    uint32_t max_move_time_ms;

    uint16_t home_deg;
    uint16_t curr_deg;

    bool is_moving;
    int32_t target_pos_continuous;
    bool target_pos_reached;

    // TODO: this is cursed
    bool is_sethome_req;
    bool return_home_after_move;
    bool desired_armed_state;

    // Internal state tracking, safe to modify in interrupts
    volatile bool connected;
    volatile bool move_active;
    volatile bool enabled;
    volatile bool homed;
    volatile bool hardware_err;
    volatile uint8_t num_errors;

    // Only valid when move_active is true
    int16_t target_deg;
    uint16_t target_pos;
    absolute_time_t move_timeout;
    uint16_t curr_move_time_ms;

    int32_t absolute_pos;
    int16_t last_position;
    int16_t curr_position;

    bool is_homing;
    bool needs_sync;
    bool flash_write_pending;
    uint8_t still_count;

    // Alarms and timers
    alarm_id_t move_complete_alarm;
    alarm_id_t start_go_home_alarm;
} servo_t;

typedef struct ServoPacket {
    servo_t *servo;
    uint8_t command_length;
    uint8_t command;
    uint8_t param_buf[PARAMETER_MTU];
    uint8_t checksum;
    servo_read_cb on_read;
} ServoPacket_t;

extern ServoPacket_t make_servo_packet(servo_t *servo, uint8_t command, uint8_t command_length,
                                       uint8_t param_buf[PARAMETER_MTU]);

extern bool enqueue_packet(ServoPacket_t packet);

extern bool uart_scheduler(repeating_timer_t *rt);

extern void send_packet(ServoPacket_t packet);

#endif  // HIWONDER_DRIVER_H
