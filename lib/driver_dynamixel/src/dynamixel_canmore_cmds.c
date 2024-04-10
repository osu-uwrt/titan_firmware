#include "dynamixel_canmore_cmds.h"

#if TITAN_DEBUG_PRESENT
#include "dynamixel_schedule.h"

#include "titan/debug.h"

#include <string.h>

static int parse_int(const char *str, long long *val_out) {
    char *end;
    long long val = strtoll(str, &end, 10);
    if (*end != 0 || end == str) {
        return 1;
    }
    *val_out = val;
    return 0;
}

#define parse_int_with_bounds(str, var_out, min_bounds, max_bounds)                                                    \
    do {                                                                                                               \
        long long val_tmp;                                                                                             \
        if (parse_int(str, &val_tmp)) {                                                                                \
            fprintf(fout, "Invalid Decimal Value Specified: '%s'\n", str);                                             \
            return 1;                                                                                                  \
        }                                                                                                              \
        if (val_tmp > (max_bounds) || val_tmp < (min_bounds)) {                                                        \
            fprintf(fout, "Provided number '%s' out of bounds! Must be between %lld - %lld\n", str,                    \
                    (long long) (min_bounds), (long long) (max_bounds));                                               \
        }                                                                                                              \
        (var_out) = val_tmp;                                                                                           \
    } while (0)

#define parse_u8(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT8_MAX)
#define parse_u16(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT16_MAX)
#define parse_u32(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT32_MAX)
#define parse_i8(str, var_out) parse_int_with_bounds(str, var_out, INT8_MIN, INT8_MAX)
#define parse_i16(str, var_out) parse_int_with_bounds(str, var_out, INT16_MIN, INT16_MAX)
#define parse_i32(str, var_out) parse_int_with_bounds(str, var_out, INT32_MIN, INT32_MAX)

static int dynwrite_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 5) {
        fprintf(fout, "Usage: dynwrite [id] [addr] [u8/s8/u16/s16/u32/s32/bytes] [data...]\n");
        return 1;
    }

    uint8_t id;
    uint16_t addr;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);
    parse_u16(argv[2], addr);

    union {
        int8_t i8;
        uint8_t u8;
        int16_t i16;
        uint16_t u16;
        int32_t i32;
        uint32_t u32;
        uint8_t bytes[4];  // Max length of any dynamixel packet is 4 bytes
    } write_data;
    size_t write_len;

    if (!strcmp(argv[3], "u8")) {
        parse_u8(argv[4], write_data.u8);
        write_len = 1;
    }
    else if (!strcmp(argv[3], "s8")) {
        parse_i8(argv[4], write_data.i8);
        write_len = 1;
    }
    else if (!strcmp(argv[3], "u16")) {
        parse_u16(argv[4], write_data.u16);
        write_len = 2;
    }
    else if (!strcmp(argv[3], "s16")) {
        parse_i16(argv[4], write_data.i16);
        write_len = 2;
    }
    else if (!strcmp(argv[3], "u32")) {
        parse_u32(argv[4], write_data.u32);
        write_len = 4;
    }
    else if (!strcmp(argv[3], "s32")) {
        parse_i32(argv[4], write_data.i32);
        write_len = 4;
    }
    else {
        fprintf(fout, "Invalid Write Mode: '%s'\n", argv[3]);
        return 1;
    }

    fprintf(fout, "Writing %d bytes starting at %d to dynamixel %d\n", write_len, addr, id);
    dynamixel_schedule_write_packet(id, addr, write_data.bytes, write_len);

    return 0;
}

static int dynram_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc < 2) {
        fprintf(fout, "Usage: dynram [id]\n");
        return 1;
    }

    uint8_t id;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);

    volatile struct dynamixel_ram *ram = dynamixel_get_ram(id);
    if (ram == NULL) {
        fprintf(fout, "Dynamixel ID %d is not monitored by firmware or is not currently connected\n", id);
        return 1;
    }

    fprintf(fout, "--- Servo %d RAM ---\n", id);
    fprintf(fout, "  Torque Enable: %d\n", ram->torque_enable);
    fprintf(fout, "  Moving: %d\n", ram->moving);
    fprintf(fout, "  Realtime Tick: %d\n", ram->realtime_tick);
    fprintf(fout, "  Hardware Error Status: %d\n", ram->hardware_error_status);
    fprintf(fout, "  Goal Position: %ld\n", ram->goal_position);
    fprintf(fout, "  Present Position: %ld\n", ram->present_position);
    fprintf(fout, "  Present Velocity: %ld\n", ram->present_velocity);
    fprintf(fout, "  Present Voltage: %d.%d V\n", ram->present_input_voltage / 10, ram->present_input_voltage % 10);
    fprintf(fout, "  Present Temperature: %d C\n", ram->present_temperature);
    fprintf(fout, "-------------------\n");
    return 0;
}

static int dyneeprom_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc < 2) {
        fprintf(fout, "Usage: dyneeprom [id]\n");
        return 1;
    }

    uint8_t id;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);

    struct dynamixel_eeprom *eeprom = dynamixel_get_eeprom(id);
    if (eeprom == NULL) {
        fprintf(fout, "Dynamixel ID %d is monitored by firmware or is not currently connected\n", id);
        return 1;
    }

    fprintf(fout, "--- Servo %d EEPROM ---\n", id);
    fprintf(fout, "  Model Number: %d\n", eeprom->model_num);
    fprintf(fout, "  Model Info: %ld\n", eeprom->model_info);
    fprintf(fout, "  Firmware Version: %d\n", eeprom->firmware_version);
    fprintf(fout, "  ID: %d\n", eeprom->id);
    fprintf(fout, "  Baud Rate: %d\n", eeprom->baud_rate);
    fprintf(fout, "  Return Delay Time: %d\n", eeprom->return_delay_time);
    fprintf(fout, "  Drive Mode: %d\n", eeprom->drive_mode);
    fprintf(fout, "  Operating Mode: %d\n", eeprom->operating_mode);
    fprintf(fout, "  Secondary ID: %d\n", eeprom->secondary_id);
    fprintf(fout, "  Protocol: %d\n", eeprom->protocol_type);
    fprintf(fout, "  Homing Offset: %ld\n", eeprom->homing_offset);
    fprintf(fout, "  Temperature Limit: %d\n", eeprom->temperature_limit);
    fprintf(fout, "  Min Voltage Limit: %d\n", eeprom->min_voltage_limit);
    fprintf(fout, "  Max Voltage Limit: %d\n", eeprom->max_voltage_limit);
    fprintf(fout, "  PWM Limit: %d\n", eeprom->pwm_limit);
    fprintf(fout, "  Velocity Limit: %ld\n", eeprom->velocity_limit);
    fprintf(fout, "  Min Position Limit: %ld\n", eeprom->min_position_limit);
    fprintf(fout, "  Max Position Limit: %ld\n", eeprom->max_position_limit);
    fprintf(fout, "  Startup Config: %d\n", eeprom->startup_config);
    fprintf(fout, "-------------------\n");
    return 0;
}

void dynamixel_canmore_cmds_register(void) {
    debug_remote_cmd_register("dynwrite", "[id] [addr] [u8/s8/u16/s16/u32/s32] [value]",
                              "Writes the data to the requested address\n"
                              "Note: All numbers are in decimal"
                              "Example: dynwrite 4 7 u8 5\n"
                              "\tWrites ID register (7) of dynamixel 4, changing its ID to 5",
                              dynwrite_cmd_cb);
    debug_remote_cmd_register("dynram", "[id]",
                              "Dumps the current RAM state of the requested dynamixel\n"
                              "Note this can only read dynamixels that are registered for monitoring the application",
                              dynram_cmd_cb);
    debug_remote_cmd_register("dyneeprom", "[id]",
                              "Dumps the EEPROM of the requested dynamixel\n"
                              "Note this can only read dynamixels that are registered for monitoring the application",
                              dyneeprom_cmd_cb);
}

#else

void dynamixel_canmore_cmds_register(void) {}

#endif
