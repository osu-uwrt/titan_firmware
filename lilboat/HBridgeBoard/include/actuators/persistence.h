#include <stdbool.h>
#include <stdint.h>

#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#define FLASH_SIZE_BYTES 2097152
#define FLASH_SECTOR_BYTES 4096
#define FLASH_OFFSET FLASH_SIZE_BYTES - FLASH_SECTOR_BYTES
#define PAGE_SIZE_BYTES 256
#define XIP_BASE_ADDRESS 0x10000000
#define SERVO_POSITION_DATA_MARKER 1

#define FLASH_STORAGE_SLOTS 6

typedef struct servo servo_t;

typedef struct {
    uint8_t id;
    int32_t absolute_pos;
} servo_persistent_data;

typedef struct {
    uint32_t servo_position_marker;
    servo_persistent_data servo_info[FLASH_STORAGE_SLOTS];
} flash_config_t;

extern flash_config_t servo_config;

bool update_servo_persistent_position(servo_t *servo, flash_config_t *config);

void read_servo_persistent_position(servo_t *servo, flash_config_t *config);

bool read_from_flash(flash_config_t *data);

void write_to_flash(flash_config_t *config);

#endif
