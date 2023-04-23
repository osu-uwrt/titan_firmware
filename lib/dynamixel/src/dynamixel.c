
#include "dynamixel.h"
#include "basic_queue/queue.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include <assert.h>

#include "pico/time.h"
#include <string.h>

#define MAX_SERVO_CNT 8
#define MAX_CMDS 8

#define PacketGetU16(array, idx)                                               \
  (((uint16_t)array[idx]) | (((uint16_t)array[idx + 1]) << 8))
#define PacketGetU32(array, idx)                                               \
  (((uint32_t)array[idx]) | (((uint32_t)array[idx + 1]) << 8) |                \
   (((uint32_t)array[idx + 2]) << 16) | (((uint32_t)array[idx + 3]) << 24))

enum internal_state {
  UNINITIALIZED,
  STATE_PING,
  /* On setup, reads the configuration of each servo. */
  STATE_EEPROM_READ,
  /** Reading all of the servo positions */
  STATE_READING,
  /** Sending commands to the servos */
  STATE_WRITING,
  /** Dynamixel driver failed */
  STATE_FAILED,
};

struct internal_cmd {
  InfoToMakeDXLPacket_t packet;
  uint8_t packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
};

static struct QUEUE_DEFINE(struct internal_cmd, MAX_CMDS) cmd_queue = {0};

static uint8_t internal_packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
static InfoToMakeDXLPacket_t internal_packet;

enum internal_state state = UNINITIALIZED;

dynamixel_error_cb error_cb;
dynamixel_event_cb event_cb;

dynamixel_id servos[MAX_SERVO_CNT];
struct dynamixel_eeprom servo_eeproms[MAX_SERVO_CNT] = {0};
struct dynamixel_ram servo_ram[MAX_SERVO_CNT] = {0};
int servo_cnt;
int current_servo_idx;

static void send_next_cmd();
static void send_ping();
static void send_eeprom_read();
static void send_ram_read();

/* ========================
 *  UTILITY FUNCTIONS
 *  ========================
 */

static int servo_id_to_index(dynamixel_id id) {
  for (int i = 0; i < servo_cnt; i++) {
    if (servos[i] == id) {
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
                                     struct dynamixel_req_result *result) {
  if (error) {
    state = STATE_FAILED;
    error_cb(error);
    return;
  }

  (void)result;

  send_next_cmd();
}

static void send_next_cmd() {
  if (QUEUE_EMPTY(&cmd_queue)) {
    current_servo_idx = 0;
    // send_ram_read(); TODO: Read RAM after like 5 ms
    return;
  }

  state = STATE_WRITING;

  struct internal_cmd *entry = QUEUE_CUR_READ_ENTRY(&cmd_queue);
  dynamixel_send_packet(on_internal_cmd_complete, &entry->packet);
  QUEUE_MARK_READ_DONE(&cmd_queue);
}

/* ========================
 *  RAM READ
 *  ========================
 */

static void on_dynamixel_ram_read(enum dynamixel_error error,
                                  struct dynamixel_req_result *result) {
  if (error) {
    state = STATE_FAILED;
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

  event_cb(DYNAMIXEL_EVENT_RAM_READ, current_servo_idx);
  current_servo_idx++;

  if (current_servo_idx < servo_cnt) {
    send_ram_read();
  } else {
    current_servo_idx = 0;
    send_next_cmd();
  }
}

static void send_ram_read() {
  if (dynamixel_create_read_packet(
          &internal_packet, internal_packet_buf, servos[current_servo_idx],
          DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_RAM_LENGTH) != DXL_LIB_OK) {
    state = STATE_FAILED;
    error_cb(DYNAMIXEL_DRIVER_ERROR);
    return;
  }

  dynamixel_send_packet(on_dynamixel_ram_read, &internal_packet);
}

/* ========================
 *  EEPROM READ
 *  ========================
 */

static void on_dynamixel_eeprom_read(enum dynamixel_error error,
                                     struct dynamixel_req_result *result) {
  if (error) {
    state = STATE_FAILED;
    error_cb(error);
    return;
  }

  uint8_t *param_buf = result->packet->p_param_buf;
  struct dynamixel_eeprom *eeprom = &servo_eeproms[current_servo_idx];
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

  event_cb(DYNAMIXEL_EVENT_EEPROM_READ, servos[current_servo_idx]);
  current_servo_idx++;

  if (current_servo_idx < servo_cnt) {
    send_eeprom_read();
  } else {
    current_servo_idx = 0;
    send_eeprom_read();
  }
}

static void send_eeprom_read() {
  if (dynamixel_create_read_packet(
          &internal_packet, internal_packet_buf, servos[current_servo_idx],
          DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_EEPROM_LENGTH) != DXL_LIB_OK) {
    state = STATE_FAILED;
    error_cb(DYNAMIXEL_DRIVER_ERROR);
    return;
  }

  dynamixel_send_packet(on_dynamixel_eeprom_read, &internal_packet);
}

/* ========================
 *  PING
 *  ========================
 */

static void on_dynamixel_ping(enum dynamixel_error error,
                              struct dynamixel_req_result *result) {
  if (error) {
    state = STATE_FAILED;
    error_cb(error);
    return;
  }

  event_cb(DYNAMIXEL_EVENT_PING, current_servo_idx);
  (void)result;
  current_servo_idx++;

  if (current_servo_idx < servo_cnt) {
    send_ping();
  } else {
    current_servo_idx = 0;
    send_eeprom_read();
  }
}

static void send_ping() {
  if (dynamixel_create_ping_packet(&internal_packet, internal_packet_buf,
                                   servos[current_servo_idx]) != DXL_LIB_OK) {
    state = STATE_FAILED;
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
                    dynamixel_event_cb _event_cb) {
  assert(state == UNINITIALIZED);
  assert(id_cnt < MAX_SERVO_CNT);
  error_cb = _error_cb;
  event_cb = _event_cb;
  servo_cnt = id_cnt;

  // copy the list of IDs
  for (size_t i = 0; i < id_cnt; i++) {
    servos[i] = id_list[i];
  }

  state = STATE_PING;
  current_servo_idx = 0;
  send_ping();
}

bool dynamixel_set_id(dynamixel_id old, dynamixel_id new) {
  if (QUEUE_FULL(&cmd_queue)) {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  dynamixel_create_write_packet(&entry->packet, entry->packet_buf, old,
                                DYNAMIXEL_CTRL_TABLE_ID_ADDR, &new, 1);
  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  return true;
}

bool dynamixel_enable_torque(dynamixel_id id, bool enabled) {
  if (QUEUE_FULL(&cmd_queue)) {
    error_cb(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
    return false;
  }

  uint8_t data = enabled ? 1 : 0;
  struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
  dynamixel_create_write_packet(&entry->packet, entry->packet_buf, id,
                                DYNAMIXEL_CTRL_TABLE_ID_ADDR, &data, 1);
  QUEUE_MARK_WRITE_DONE(&cmd_queue);

  return true;
}

bool dynamixel_set_target_position(dynamixel_id id, uint32_t pos) {
  assert(false);
  return false;
}

void dynamixel_get_eeprom(dynamixel_id id, struct dynamixel_eeprom *eeprom) {
  int idx = servo_id_to_index(id);
  assert(idx >= 0); // TODO: what to do in this case?

  memcpy(eeprom, &servo_eeproms[idx], sizeof(struct dynamixel_eeprom));
}

void dynamixel_get_ram(dynamixel_id id, struct dynamixel_ram *ram) {
  int idx = servo_id_to_index(id);
  assert(idx >= 0); // TODO: what to do in this case?

  memcpy(ram, &servo_ram[idx], sizeof(struct dynamixel_ram));
}
