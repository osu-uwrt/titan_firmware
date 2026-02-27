#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "hiwonder_driver.h"

#include "pico/stdlib.h"

#define NUM_SERVOS 2

#define SERVO_MIN_DEG 0
#define SERVO_MAX_DEG 240

#define UNITS_PER_DEGREE 65536.0f / 360.0f

#define FLASH_SIZE_BYTES 2097152
#define FLASH_SECTOR_BYTES 4096
#define FLASH_OFFSET FLASH_SIZE_BYTES - FLASH_SECTOR_BYTES
#define PAGE_SIZE_BYTES 256
#define XIP_BASE_ADDRESS 0x10000000
#define ROBOT_MAGIC_NUMBER 1

typedef struct {
    uint32_t magic_number;
    int32_t abs_position[NUM_SERVOS];
} config_t;

extern config_t servo_config;

extern void servo_ping(servo_t *servo);

extern void servo_set_armed(servo_t *servo, bool armed);

extern uint16_t servo_set_deg(servo_t *servo, float deg);

extern void servo_set_deg_then_home(servo_t *servo, float deg);

extern void servo_read_deg(servo_t *servo);

extern void servo_continuous_move_ms(servo_t *servo, int16_t speed, uint32_t ms);

extern void servo_continuous_set_deg(servo_t *servo, int16_t speed, float deg);

extern void servo_continuous_move_deg(servo_t *servo, int32_t target_deg, int16_t speed);

extern void simuilate_encoder(servo_t *servo);

extern void servo_read_continuous(servo_t *servo);

extern void servo_set_id(uint8_t old_id, uint8_t new_id);

extern void servo_read_id();

extern void servo_go_home(servo_t *servo);

extern void servo_set_home(servo_t *servo);

extern void servo_init_internal();

extern void make_servo(servo_t *servo, uint8_t id, uint16_t home_deg, int32_t abs_position);

extern void write_to_flash(config_t *config);

extern bool read_from_flash(config_t *data);

#endif  // ACTUATOR_H
