
#include <assert.h>
#include <string.h>
#include "pico/time.h"

#include "dynamixel/dynamixel.h"
#include "async_uart.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include "dynamixel_schedule.h"

// PICO_CONFIG: DYNAMIXEL_TIMEOUT_INTERVAL_MS, Maximum duration of active transfer before timeout in milliseconds, min=1, default=50, group=dynamixel
#ifndef DYNAMIXEL_TIMEOUT_INTERVAL_MS
#define DYNAMIXEL_TIMEOUT_INTERVAL_MS 50
#endif

// PICO_CONFIG: DYNAMIXEL_BAUD_RATE, Baud rate of dynamixel servo comm interface, default=57600, group=dynamixel
#ifndef DYNAMIXEL_BAUD_RATE
#define DYNAMIXEL_BAUD_RATE 57600
#endif


/* ========================
 *  Public Functions
 *  ========================
 */

void dynamixel_init(PIO pio, uint sm, uint pin,
                    const dynamixel_id *id_list, size_t id_cnt,
                    dynamixel_error_cb error_cb,
                    dynamixel_event_cb event_cb)
{
  async_uart_init(pio, sm, pin, DYNAMIXEL_BAUD_RATE, DYNAMIXEL_TIMEOUT_INTERVAL_MS);
  sleep_ms(1);  // Give time for line to settle

  // Initialize the dynamixel command scheduler
  dynamixel_schedule_init(id_list, id_cnt, error_cb, event_cb);
}

void dynamixel_set_id(dynamixel_id old, dynamixel_id new)
{
  dynamixel_schedule_write_packet(old, DYNAMIXEL_CTRL_TABLE_ID_ADDR, &new, sizeof(new));
}

void dynamixel_enable_torque(dynamixel_id id, bool enabled)
{
  uint8_t data = enabled ? 1 : 0;
  dynamixel_schedule_write_packet(id, DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR, &data, sizeof(data));
}

void dynamixel_set_target_position(dynamixel_id id, int32_t pos)
{
  uint8_t byte1 = pos & 0xFF;
  uint8_t byte2 = (pos >> 8) & 0xFF;
  uint8_t byte3 = (pos >> 16) & 0xFF;
  uint8_t byte4 = (pos >> 24) & 0xFF;
  uint8_t data[] = {byte1, byte2, byte3, byte4};
  // TODO: create functions called dynamixel_create_writeu32_packet and other variants
  dynamixel_schedule_write_packet(id, DYNAMIXEL_CTRL_TABLE_GOAL_POSITION_ADDR, data, sizeof(data));
}

bool dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom)
{
  struct dynamixel_state* state = dynamixel_schedule_get_state_ptr(id);

  if (state == NULL || !state->connected) {
    return false;
  }

  // TODO: Do we want this function?

  memcpy(eeprom, &state->ram, sizeof(struct dynamixel_eeprom));
  return true;
}

bool dynamixel_get_ram(dynamixel_id id, struct dynamixel_ram *ram)
{
  struct dynamixel_state* state = dynamixel_schedule_get_state_ptr(id);

  if (state == NULL || state->connected) {
    return false;
  }

  // TODO: Do we want this function?

  memcpy(ram, &state->ram, sizeof(struct dynamixel_ram));
  return true;
}

bool dynamixel_check_connected(dynamixel_id id) {
  struct dynamixel_state* state = dynamixel_schedule_get_state_ptr(id);

  if (state == NULL) {
    return false;
  }

  return state->connected;
}

bool dynamixel_get_position(dynamixel_id id, int32_t *position_out) {
  struct dynamixel_state* state = dynamixel_schedule_get_state_ptr(id);

  if (state == NULL || !state->connected) {
    return false;
  }

  *position_out = state->ram.present_position;
  return true;
}
