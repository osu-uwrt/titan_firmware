
#include "driver/dynamixel.h"

#include "async_uart.h"
#include "dynamixel_canmore_cmds.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include "dynamixel_schedule.h"

#include "pico/time.h"

#include <assert.h>
#include <string.h>

// PICO_CONFIG: DYNAMIXEL_TIMEOUT_INTERVAL_MS, Maximum duration of active transfer before timeout in milliseconds, min=1, default=50, group=driver_dynamixel
#ifndef DYNAMIXEL_TIMEOUT_INTERVAL_MS
#define DYNAMIXEL_TIMEOUT_INTERVAL_MS 50
#endif

// PICO_CONFIG: DYNAMIXEL_BAUD_RATE, Baud rate of dynamixel servo comm interface, default=57600, group=driver_dynamixel
#ifndef DYNAMIXEL_BAUD_RATE
#define DYNAMIXEL_BAUD_RATE 57600
#endif

/* ========================
 *  Register Write Functions
 *  ========================
 */

static inline void dynamixel_write_u8(dynamixel_id id, uint16_t reg, uint8_t data) {
    dynamixel_schedule_write_packet(id, reg, &data, sizeof(data));
}

static inline void dynamixel_write_u16(dynamixel_id id, uint16_t reg, uint16_t data) {
    uint8_t byte1 = data & 0xFF;
    uint8_t byte2 = (data >> 8) & 0xFF;
    uint8_t data_arr[] = { byte1, byte2 };
    dynamixel_schedule_write_packet(id, reg, data_arr, sizeof(data_arr));
}

static inline void dynamixel_write_u32(dynamixel_id id, uint16_t reg, uint32_t data) {
    uint8_t byte1 = data & 0xFF;
    uint8_t byte2 = (data >> 8) & 0xFF;
    uint8_t byte3 = (data >> 16) & 0xFF;
    uint8_t byte4 = (data >> 24) & 0xFF;
    uint8_t data_arr[] = { byte1, byte2, byte3, byte4 };
    dynamixel_schedule_write_packet(id, reg, data_arr, sizeof(data_arr));
}

static inline void dynamixel_write_s8(dynamixel_id id, uint16_t reg, int8_t data) {
    dynamixel_write_u8(id, reg, (uint8_t) data);
}

static inline void dynamixel_write_s16(dynamixel_id id, uint16_t reg, int16_t data) {
    dynamixel_write_u16(id, reg, (uint16_t) data);
}

static inline void dynamixel_write_s32(dynamixel_id id, uint16_t reg, int32_t data) {
    dynamixel_write_u32(id, reg, (uint32_t) data);
}

/* ========================
 *  Public Functions
 *  ========================
 */

void dynamixel_init(PIO pio, uint sm, uint pin, const dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb,
                    dynamixel_event_cb event_cb) {
    async_uart_init(pio, sm, pin, DYNAMIXEL_BAUD_RATE, DYNAMIXEL_TIMEOUT_INTERVAL_MS);
    sleep_ms(1);  // Give time for line to settle

    // Initialize the dynamixel command scheduler
    dynamixel_schedule_init(id_list, id_cnt, error_cb, event_cb);

    // Register canmore commands for debug access
    dynamixel_canmore_cmds_register();
}

void dynamixel_request_eeprom_rescan(dynamixel_id id) {
    dynamixel_schedule_eeprom_read(id);
}

/* ========================
 *  Setter Functions
 *  ========================
 */

void dynamixel_set_id(dynamixel_id old, dynamixel_id new) {
    dynamixel_write_u8(old, DYNAMIXEL_CTRL_TABLE_ID_ADDR, new);
}

void dynamixel_enable_torque(dynamixel_id id, bool enabled) {
    dynamixel_write_u8(id, DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR, enabled ? 1 : 0);
}

void dynamixel_set_target_position(dynamixel_id id, int32_t pos) {
    dynamixel_write_s32(id, DYNAMIXEL_CTRL_TABLE_GOAL_POSITION_ADDR, pos);
}

void dynamixel_set_homing_offset(dynamixel_id id, int32_t home_offset) {
    dynamixel_write_s32(id, DYNAMIXEL_CTRL_TABLE_HOMING_OFFSET_ADDR, home_offset);
}

/* ========================
 *  Getter Functions
 *  ========================
 */

struct dynamixel_eeprom *dynamixel_get_eeprom(dynamixel_id id) {
    struct dynamixel_state *state = dynamixel_schedule_get_state_ptr(id);

    if (state == NULL || !state->connected) {
        return NULL;
    }

    return &state->eeprom;
}

volatile struct dynamixel_ram *dynamixel_get_ram(dynamixel_id id) {
    struct dynamixel_state *state = dynamixel_schedule_get_state_ptr(id);

    if (state == NULL || !state->connected) {
        return NULL;
    }

    return &state->ram;
}

bool dynamixel_check_connected(dynamixel_id id) {
    struct dynamixel_state *state = dynamixel_schedule_get_state_ptr(id);

    if (state == NULL) {
        return false;
    }

    return state->connected;
}

bool dynamixel_get_position(dynamixel_id id, int32_t *position_out) {
    struct dynamixel_state *state = dynamixel_schedule_get_state_ptr(id);

    if (state == NULL || !state->connected) {
        return false;
    }

    *position_out = state->ram.present_position;
    return true;
}
