
#include <assert.h>
#include <string.h>
#include "pico/time.h"
#include "basic_queue/queue.h"

#include "dynamixel/async_uart.h"
#include "dynamixel/dynamixel.h"
#include "dynamixel_controls.h"
#include "dynamixel_comms.h"

#define MAX_SERVO_CNT 8
#define MAX_CMDS 8
#define READ_INTERVAL_MS 4000

#define PacketGetU16(array, idx) \
  (((uint16_t)array[idx]) | (((uint16_t)array[idx + 1]) << 8))
#define PacketGetU32(array, idx)                                \
  (((uint32_t)array[idx]) | (((uint32_t)array[idx + 1]) << 8) | \
   (((uint32_t)array[idx + 2]) << 16) | (((uint32_t)array[idx + 3]) << 24))

enum internal_cmd_type {
  CMD_TYPE_NONE = 0,
  /* No extra processing needs to be completed */
  CMD_TYPE_GENERAL = 1,
  CMD_TYPE_EEPROM_READ = 2,
};

struct internal_cmd
{
  InfoToMakeDXLPacket_t packet;
  uint8_t packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
  enum internal_cmd_type cmd_type;
};

static struct QUEUE_DEFINE(struct internal_cmd, MAX_CMDS) cmd_queue = {0};
enum internal_cmd_type current_cmd_type = CMD_TYPE_NONE;
bool ram_read_flag = false;

static uint8_t internal_packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
static InfoToMakeDXLPacket_t internal_packet;

dynamixel_error_cb error_cb;
dynamixel_event_cb event_cb;

dynamixel_id servos[MAX_SERVO_CNT];
struct dynamixel_eeprom servo_eeproms[MAX_SERVO_CNT] = {0};
struct dynamixel_ram servo_ram[MAX_SERVO_CNT] = {0};
int servo_cnt;
int current_servo_idx;

static void send_next_cmd();
static void send_ping();
static void parse_eeprom_read(struct dynamixel_eeprom *eeprom, uint8_t *param_buf);
static void send_eeprom_read();
static void send_ram_read();

/* ========================
 *  UTILITY FUNCTIONS
 *  ========================
 */

static int servo_id_to_index(dynamixel_id id)
{
  for (int i = 0; i < servo_cnt; i++)
  {
    if (servos[i] == id)
    {
      return i;
    }
  }

  return -1;
}

/* ========================
 *  INTERNAL CMDS
 *  ========================
 */

static void on_internal_cmd_complete(enum dynamixel_error error,
                                     struct dynamixel_req_result *result)
{
  if (error)
  {
    error_cb(error);
    return;
  }

  dynamixel_id id = result->packet->id;
  if (current_cmd_type == CMD_TYPE_EEPROM_READ) {
    int index = servo_id_to_index(id);
    if (index < 0) {
      error_cb(DYNAMIXEL_INVALID_ID);
    } else {
      struct dynamixel_eeprom *eeprom = &servo_eeproms[index];
      parse_eeprom_read(eeprom, result->packet->p_param_buf);
      event_cb(DYNAMIXEL_EVENT_EEPROM_READ, id);
    }
  }
  (void)result;

  current_cmd_type = CMD_TYPE_NONE;
  send_next_cmd();
}

static void send_next_cmd()
{
  assert(current_cmd_type == CMD_TYPE_NONE); // TODO: convert this to an error callback

  if(ram_read_flag) {
    current_servo_idx = 0;
    send_ram_read();
    return;
  }

  if (QUEUE_EMPTY(&cmd_queue))
  {
    current_cmd_type = CMD_TYPE_NONE;
    return;
  }

  struct internal_cmd *entry = QUEUE_CUR_READ_ENTRY(&cmd_queue);
  current_cmd_type = entry->cmd_type;
  dynamixel_send_packet(on_internal_cmd_complete, &entry->packet);
  QUEUE_MARK_READ_DONE(&cmd_queue);
}

static int64_t on_ram_read_alarm(alarm_id_t id, void *user_data) {
  (void) id;
  (void) user_data;

  ram_read_flag = true;

  if (current_cmd_type == CMD_TYPE_NONE) {
    current_servo_idx = 0;
    send_ram_read();
  }

  return 0;
}

/* ========================
 *  RAM READ
 *  ========================
 */

static void on_dynamixel_ram_read(enum dynamixel_error error,
                                  struct dynamixel_req_result *result)
{
  assert(ram_read_flag);

  if (error)
  {
    error_cb(error);
    return;
  }

  uint8_t *param_buf = result->packet->p_param_buf;
  struct dynamixel_ram *ram = &servo_ram[current_servo_idx];
  ram->torque_enable = param_buf[DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR -
                                 DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR];
  ram->hardware_error_status =
      param_buf[DYNAMIXEL_CTRL_TABLE_HARDWARE_ERROR_STATUS_ADDR -
                DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR];
  ram->present_position =
      PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_POSITION_ADDR -
                                  DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
  ram->present_velocity =
      PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_VELOCITY_ADDR -
                                  DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
  ram->goal_position = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_GOAL_POSITION_ADDR - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
  // ram-> = param_buf[DYNAMIXEL_CTRL_TABLE_HARDWARE_ERROR_STATUS_ADDR -
  // DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR] bool torque_enable; uint8_t led;
  // uint8_t status_return_level;
  // uint8_t registered_instruction;
  // uint8_t hardware_error_status;
  // uint16_t velocity_i_gain;
  // uint16_t velocity_p_gain;
  // uint16_t position_d_gain;
  // uint16_t position_i_gain;
  // uint16_t position_p_gain;
  // uint16_t feedforward_2nd_gain;
  // uint16_t feedforward_1st_gain;
  // uint8_t bus_watchdog;
  // uint16_t goal_pwm;
  // uint32_t goal_velocity;
  // uint32_t profile_accel;
  // uint32_t profile_vel;
  // uint32_t goal_position;
  // uint16_t realtime_tick;
  // uint8_t moving;
  // uint8_t moving_status;
  // uint16_t present_pwm;
  // uint16_t present_load;
  // uint32_t present_velocity;
  // uint32_t present_position;
  // uint32_t velocity_trajectory;
  // uint32_t position_trajectory;
  // uint16_t present_input_voltage;
  // uint16_t present_temperature;
  // uint8_t backup_ready;

  event_cb(DYNAMIXEL_EVENT_RAM_READ, servos[current_servo_idx]);
  current_servo_idx++;

  if (current_servo_idx < servo_cnt)
  {
    send_ram_read();
  }
  else
  {
    ram_read_flag = false;
    add_alarm_in_ms(READ_INTERVAL_MS, on_ram_read_alarm, NULL, true);
    send_next_cmd();
  }
}

static void send_ram_read()
{
  assert(ram_read_flag);

  if (dynamixel_create_read_packet(
          &internal_packet, internal_packet_buf, servos[current_servo_idx],
          DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_RAM_LENGTH) != DXL_LIB_OK)
  {
    error_cb(DYNAMIXEL_DRIVER_ERROR);
    return;
  }

  dynamixel_send_packet(on_dynamixel_ram_read, &internal_packet);
}

/* ========================
 *  EEPROM READ
 *  ========================
 */

static void parse_eeprom_read(struct dynamixel_eeprom *eeprom, uint8_t *param_buf) {
  // TODO: need to subtract the eeprom start address in case it does not start
  // at zero
  eeprom->model_num =
      PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_MODEL_NUM_ADDR);
  eeprom->model_info =
      PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MODEL_INFO_ADDR);
  eeprom->firmware_version =
      param_buf[DYNAMIXEL_CTRL_TABLE_FIRMWARE_VERSION_ADDR];
  eeprom->id = param_buf[DYNAMIXEL_CTRL_TABLE_ID_ADDR];
  eeprom->baud_rate = param_buf[DYNAMIXEL_CTRL_TABLE_BAUD_RATE_ADDR];
  eeprom->drive_mode = param_buf[DYNAMIXEL_CTRL_TABLE_DRIVE_MODE_ADDR];
  eeprom->operating_mode = param_buf[DYNAMIXEL_CTRL_TABLE_OPERATE_MODE_ADDR];
  eeprom->startup_config = param_buf[DYNAMIXEL_CTRL_TABLE_STARTUP_CONFIG_ADDR];
  eeprom->min_position_limit = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MIN_POSITION_LIMIT_ADDR);
  eeprom->max_position_limit = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MAX_POSITION_LIMIT_ADDR);

  // TODO: Read these
  // uint8_t return_delay_time;
  // uint8_t secondary_id;
  // uint8_t protocol_type;
  // uint32_t homing_offset;
  // uint8_t temperature_limit;
  // uint16_t max_voltage_limit;
  // uint16_t min_voltage_limit;
  // uint16_t pwm_limit;
  // uint32_t velocity_limit;
  // uint32_t max_position_limit;
  // uint32_t min_position_limit;
}

static void on_dynamixel_eeprom_read(enum dynamixel_error error,
                                     struct dynamixel_req_result *result)
{
  if (error)
  {
    error_cb(error);
    return;
  }

  struct dynamixel_eeprom *eeprom = &servo_eeproms[current_servo_idx];
  parse_eeprom_read(eeprom, result->packet->p_param_buf);
  event_cb(DYNAMIXEL_EVENT_EEPROM_READ, servos[current_servo_idx]);
  current_servo_idx++;

  if (current_servo_idx < servo_cnt)
  {
    send_eeprom_read();
  }
  else
  {
    current_servo_idx = 0;
    ram_read_flag = true;
    send_ram_read();
  }
}

static void send_eeprom_read()
{
  if (dynamixel_create_read_packet(
          &internal_packet, internal_packet_buf, servos[current_servo_idx],
          DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_EEPROM_LENGTH) != DXL_LIB_OK)
  {
    error_cb(DYNAMIXEL_DRIVER_ERROR);
  }

  dynamixel_send_packet(on_dynamixel_eeprom_read, &internal_packet);
}

/* ========================
 *  PING
 *  ========================
 */

static void on_dynamixel_ping(enum dynamixel_error error,
                              struct dynamixel_req_result *result)
{
  if (error)
  {
    error_cb(error);
    return;
  }

  event_cb(DYNAMIXEL_EVENT_PING, current_servo_idx);
  (void)result;
  current_servo_idx++;

  if (current_servo_idx < servo_cnt)
  {
    send_ping();
  }
  else
  {
    current_servo_idx = 0;
    send_eeprom_read();
  }
}

static void send_ping()
{
  if (dynamixel_create_ping_packet(&internal_packet, internal_packet_buf,
                                   servos[current_servo_idx]) != DXL_LIB_OK)
  {
    error_cb(DYNAMIXEL_DRIVER_ERROR);
    return;
  }

  dynamixel_send_packet(on_dynamixel_ping, &internal_packet);
}

/* ========================
 *  USER FUNCTIONS BELOW
 *  ========================
 */

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt,
                    dynamixel_error_cb _error_cb,
                    dynamixel_event_cb _event_cb)
{
  assert(id_cnt < MAX_SERVO_CNT);
  async_uart_init(pio0, 0, 4, 57600, 100);
  sleep_ms(5);  // Give time for line to settle
  error_cb = _error_cb;
  event_cb = _event_cb;
  servo_cnt = id_cnt;

  // copy the list of IDs
  for (size_t i = 0; i < id_cnt; i++)
  {
    servos[i] = id_list[i];
  }

  current_servo_idx = 0;
  send_ping();
}

bool dynamixel_set_id(dynamixel_id old, dynamixel_id new)
{
  if (QUEUE_FULL(&cmd_queue))
  {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  entry->cmd_type = CMD_TYPE_GENERAL;
  dynamixel_create_write_packet(&entry->packet, entry->packet_buf, old,
                                DYNAMIXEL_CTRL_TABLE_ID_ADDR, &new, 1);
  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  if(!ram_read_flag && current_cmd_type == CMD_TYPE_NONE) {
    send_next_cmd();
  }

  return true;
}

bool dynamixel_enable_torque(dynamixel_id id, bool enabled)
{
  if (QUEUE_FULL(&cmd_queue))
  {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  uint8_t data = enabled ? 1 : 0;
  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  entry->cmd_type = CMD_TYPE_GENERAL;
  dynamixel_create_write_packet(&entry->packet, entry->packet_buf, id,
                                DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR, &data, 1);
  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  if(!ram_read_flag && current_cmd_type == CMD_TYPE_NONE) {
    send_next_cmd();
  }

  return true;
}

bool dynamixel_set_target_position(dynamixel_id id, uint32_t pos)
{
  if (QUEUE_FULL(&cmd_queue))
  {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  uint8_t byte1 = pos & 0xFF;
  uint8_t byte2 = (pos >> 8) & 0xFF;
  uint8_t byte3 = (pos >> 16) & 0xFF;
  uint8_t byte4 = (pos >> 24) & 0xFF;
  uint8_t data[] = {byte1, byte2, byte3, byte4};
  // TODO: create functions called dynamixel_create_writeu32_packet and other variants
  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  entry->cmd_type = CMD_TYPE_GENERAL;
  dynamixel_create_write_packet(&entry->packet, entry->packet_buf, id,
                                DYNAMIXEL_CTRL_TABLE_GOAL_POSITION_ADDR, data, 4);
  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  if(!ram_read_flag && current_cmd_type == CMD_TYPE_NONE) {
    send_next_cmd();
  }

  return true;
}

bool dynamixel_read_eeprom(dynamixel_id id) {
  if (QUEUE_FULL(&cmd_queue))
  {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  entry->cmd_type = CMD_TYPE_EEPROM_READ;
  if (dynamixel_create_read_packet(
          &entry->packet, entry->packet_buf, id,
          DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_EEPROM_LENGTH) != DXL_LIB_OK)
  {
    error_cb(DYNAMIXEL_DRIVER_ERROR);
    QUEUE_MARK_WRITE_DONE(&cmd_queue);
    return false;
  }

  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  if(!ram_read_flag && current_cmd_type == CMD_TYPE_NONE) {
    send_next_cmd();
  }

  return true;
}

void dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom)
{
  int idx = servo_id_to_index(id);
  assert(idx >= 0); // TODO: what to do in this case?

  memcpy(eeprom, &servo_eeproms[idx], sizeof(struct dynamixel_eeprom));
}

void dynamixel_get_ram(dynamixel_id id, struct dynamixel_ram *ram)
{
  int idx = servo_id_to_index(id);
  assert(idx >= 0); // TODO: what to do in this case?

  memcpy(ram, &servo_ram[idx], sizeof(struct dynamixel_ram));
}
