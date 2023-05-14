#include "driver/dynamixel.h"

#include "dxl_packet.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include "dynamixel_reg.h"

#define PacketGetU8(array, idx) (array[idx])
#define PacketGetU16(array, idx) \
  (((uint16_t)array[idx]) | (((uint16_t)array[idx + 1]) << 8))
#define PacketGetU32(array, idx)                                \
  (((uint32_t)array[idx]) | (((uint32_t)array[idx + 1]) << 8) | \
   (((uint32_t)array[idx + 2]) << 16) | (((uint32_t)array[idx + 3]) << 24))

#define PacketGetS8(array, idx) ((int8_t)PacketGetU8(array, idx))
#define PacketGetS16(array, idx) ((int16_t)PacketGetU16(array, idx))
#define PacketGetS32(array, idx) ((int32_t)PacketGetU32(array, idx))

// ========================
//  EEPROM READ
// ========================

void dynamixel_reg_eeprom_decode(InfoToParseDXLPacket_t *response, struct dynamixel_eeprom *eeprom) {
    uint8_t *param_buf = response->p_param_buf;

    eeprom->model_num           = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_MODEL_NUM_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->model_info          = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MODEL_INFO_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->firmware_version    = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_FIRMWARE_VERSION_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->id                  = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_ID_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->baud_rate           = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_BAUD_RATE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->return_delay_time   = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_RETURN_DELAY_TIME_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->drive_mode          = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_DRIVE_MODE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->operating_mode      = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_OPERATE_MODE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->secondary_id        = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_SECONDARY_ID_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->protocol_type       = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_PROTOCOL_TYPE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->homing_offset       = PacketGetS32(param_buf, DYNAMIXEL_CTRL_TABLE_HOMING_OFFSET_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->moving_threshold    = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MOVING_THRESHOLD_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->temperature_limit   = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_TEMPERATURE_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->max_voltage_limit   = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_MAX_VOLTAGE_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->min_voltage_limit   = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_MIN_VOLTAGE_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->pwm_limit           = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_PWM_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->velocity_limit      = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_VELOCITY_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->max_position_limit  = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MAX_POSITION_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->min_position_limit  = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_MIN_POSITION_LIMIT_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->startup_config      = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_STARTUP_CONFIG_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);
    eeprom->shutdown            = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_SHUTDOWN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR);

}

enum DXLLibErrorCode dynamixel_reg_eeprom_gen_request(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, size_t packet_buf_size, dynamixel_id id) {
    return dynamixel_create_read_packet(
          packet, packet_buf, packet_buf_size, id,
          DYNAMIXEL_CTRL_TABLE_EEPROM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_EEPROM_LENGTH);
}

// ========================
//  RAM READ
// ========================

void dynamixel_reg_ram_decode(InfoToParseDXLPacket_t *response, struct dynamixel_ram *ram) {
    uint8_t *param_buf = response->p_param_buf;

    ram->torque_enable           = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->led                     = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_LED_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->status_return_level     = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_STATUS_RETURN_LEVEL_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->registered_instruction  = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_REGISTERED_INSTRUCTION_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->hardware_error_status   = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_HARDWARE_ERROR_STATUS_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->velocity_i_gain         = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_VELOCITY_I_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->velocity_p_gain         = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_VELOCITY_P_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->position_d_gain         = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_POSITION_D_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->position_i_gain         = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_POSITION_I_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->position_p_gain         = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_POSITION_P_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->feedforward_2nd_gain    = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_FEEDFORWARD_2ND_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->feedforward_1st_gain    = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_FEEDFORWARD_1ST_GAIN_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->bus_watchdog            = PacketGetS8(param_buf, DYNAMIXEL_CTRL_TABLE_BUS_WATCHDOG_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->goal_pwm                = PacketGetS16(param_buf, DYNAMIXEL_CTRL_TABLE_GOAL_PWM_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->goal_velocity           = PacketGetS32(param_buf, DYNAMIXEL_CTRL_TABLE_GOAL_VELOCITY_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->profile_accel           = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_PROFILE_ACCELERATION_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->profile_vel             = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_PROFILE_VELOCITY_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->goal_position           = PacketGetS32(param_buf, DYNAMIXEL_CTRL_TABLE_GOAL_POSITION_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->realtime_tick           = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_REALTIME_TICK_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->moving                  = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_MOVING_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->moving_status           = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_MOVING_STATUS_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_pwm             = PacketGetS16(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_PWM_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_load            = PacketGetS16(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_LOAD_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_velocity        = PacketGetS32(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_VELOCITY_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_position        = PacketGetS32(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_POSITION_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->velocity_trajectory     = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_VELOCITY_TRAJECTORY_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->position_trajectory     = PacketGetU32(param_buf, DYNAMIXEL_CTRL_TABLE_POSITION_TRAJECTORY_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_input_voltage   = PacketGetU16(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_INPUT_VOLTAGE_ADDR
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->present_temperature     = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_PRESENT_TEMPERATURE
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
    ram->backup_ready            = PacketGetU8(param_buf, DYNAMIXEL_CTRL_TABLE_BACKUP_READY
                                     - DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR);
}

enum DXLLibErrorCode dynamixel_reg_ram_gen_request(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, size_t packet_buf_size, dynamixel_id id) {
    return dynamixel_create_read_packet(
          packet, packet_buf, packet_buf_size, id,
          DYNAMIXEL_CTRL_TABLE_RAM_START_ADDR,
          DYNAMIXEL_CTRL_TABLE_RAM_LENGTH);
}

