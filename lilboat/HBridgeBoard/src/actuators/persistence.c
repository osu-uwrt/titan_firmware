#include "actuators/persistence.h"
#include "actuators/actuator.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

flash_config_t servo_config;

bool update_servo_persistent_position(servo_t *servo, flash_config_t *config) {
    for (int i = 0; i < FLASH_STORAGE_SLOTS; i++) {
        if (config->servo_info[i].id == servo->id) {
            config->servo_info[i].absolute_pos = servo->absolute_pos;
            return true;
        }
    }

    return false;
}

void read_servo_persistent_position(servo_t *servo, flash_config_t *config) {
    for (int i = 0; i < FLASH_STORAGE_SLOTS; i++) {
        if (config->servo_info[i].id == servo->id) {
            servo->absolute_pos = config->servo_info[i].absolute_pos;
        }
    }
}

void write_to_flash(flash_config_t *config) {
    uint32_t prev_interrupt = save_and_disable_interrupts();

    flash_range_erase(FLASH_OFFSET, FLASH_SECTOR_BYTES);  // Clear last sector of flash

    uint8_t buffer[PAGE_SIZE_BYTES] = { 0 };
    config->servo_position_marker = SERVO_POSITION_DATA_MARKER;
    memcpy(buffer, config, sizeof(flash_config_t));
    flash_range_program(FLASH_OFFSET, buffer, PAGE_SIZE_BYTES);

    restore_interrupts(prev_interrupt);
    printf("Magic number: %d", buffer[0]);
}

bool read_from_flash(flash_config_t *data) {
    const flash_config_t *flash_contents = (flash_config_t *) (XIP_BASE_ADDRESS + FLASH_OFFSET);
    memcpy(data, flash_contents, sizeof(flash_config_t));

    return data->servo_position_marker == SERVO_POSITION_DATA_MARKER;
}
