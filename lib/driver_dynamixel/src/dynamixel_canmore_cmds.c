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

#define str_table_lookup(table, idx) ((idx) < (sizeof(table) / sizeof(*table)) ? table[idx] : "Unknown")
#define bool_lookup(val) ((val) ? "Yes" : "No")

static const char *const velocity_profiles[] = { "Not Used", "Rectangular", "Triangular", "Trapezoidal" };

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
    fprintf(fout, "  Torque Enable (64):\t\t%s\n", bool_lookup(ram->torque_enable));
    fprintf(fout, "  Moving (122):\t\t\t%s\n", bool_lookup(ram->moving));
    fprintf(fout, "  Moving Status (123):\t\t%d\n", ram->moving_status);
    fprintf(fout, "   - Bit 0: In-Position:\t%s\n", bool_lookup(ram->moving_status & (1 << 0)));
    fprintf(fout, "   - Bit 1: Profile Ongoing:\t%s\n", bool_lookup(ram->moving_status & (1 << 1)));
    fprintf(fout, "   - Bit 3: Following Error:\t%s\n", bool_lookup(ram->moving_status & (1 << 3)));
    fprintf(fout, "   - Bit 4-5: Velocity Profile:\t%s\n",
            str_table_lookup(velocity_profiles, (ram->moving_status >> 4) & 0x3));
    fprintf(fout, "  Realtime Tick (120):\t\t%d ms\n", ram->realtime_tick);

    fprintf(fout, "  Hardware Error Status (70):\t%d\n", ram->hardware_error_status);
    if (ram->hardware_error_status & (1 << 0))
        fprintf(fout, "    - Bit 0: Input Voltage Error\n");
    if (ram->hardware_error_status & (1 << 1))
        fprintf(fout, "    - Bit 1: Unknown???\n");
    if (ram->hardware_error_status & (1 << 2))
        fprintf(fout, "    - Bit 2: Overheating Error\n");
    if (ram->hardware_error_status & (1 << 3))
        fprintf(fout, "    - Bit 3: Motor Encoder Error\n");
    if (ram->hardware_error_status & (1 << 4))
        fprintf(fout, "    - Bit 4: Electrical Shock Error\n");
    if (ram->hardware_error_status & (1 << 5))
        fprintf(fout, "    - Bit 5: Overload Error\n");
    if (ram->hardware_error_status & (1 << 6))
        fprintf(fout, "    - Bit 6: Unknown??\n");
    if (ram->hardware_error_status & (1 << 7))
        fprintf(fout, "    - Bit 7: Unknown??\n");

    fprintf(fout, "  Goal PWM (100):\t\t%.3f%% (%d)\n", ram->goal_pwm * 0.113, ram->goal_pwm);
    fprintf(fout, "  Goal Velocity (104):\t\t%.3f rpm (%ld)\n", ram->goal_velocity * 0.229, ram->goal_velocity);
    fprintf(fout, "  Goal Position (116):\t\t%ld\n", ram->goal_position);
    fprintf(fout, "  Present PWM (124):\t\t%.3f%% (%d)\n", ram->present_pwm * 0.113, ram->present_pwm);
    fprintf(fout, "  Present Load (126):\t\t%d.%d%%\n", ram->present_load / 10, ram->present_load % 10);
    fprintf(fout, "  Present Velocity (128):\t%.3f rpm (%ld)\n", ram->present_velocity * 0.229, ram->present_velocity);
    fprintf(fout, "  Present Position (132):\t%ld\n", ram->present_position);
    fprintf(fout, "  Present Voltage (144):\t%d.%d V\n", ram->present_input_voltage / 10,
            ram->present_input_voltage % 10);
    fprintf(fout, "  Present Temperature (146):\t%d C\n", ram->present_temperature);
    fprintf(fout, "-------------------\n");
    return 0;
}

static const char *const baudrates[] = { "9,600b", "57,600b", "115,200b", "1Mb", "2Mb", "3Mb", "4Mb", "4.5Mb" };
static const char *const operating_modes[] = {
    "Unknown", "Velocity Control", "Unknown", "Position Control", "Extended Position Control",
    "Unknown", "Unknown",          "Unknown", "Unknown",          "Unknown",
    "Unknown", "Unknown",          "Unknown", "Unknown",          "Unknown",
    "Unknown", "PWM Control"
};

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
    fprintf(fout, "  Model Number (0):\t\t%d\n", eeprom->model_num);
    fprintf(fout, "  Model Info (2):\t\t%ld\n", eeprom->model_info);
    fprintf(fout, "  Firmware Version (6):\t\tv%d\n", eeprom->firmware_version);
    fprintf(fout, "  ID (7):\t\t\t%d\n", eeprom->id);
    fprintf(fout, "  Baud Rate (8):\t\t%s (%d)\n", str_table_lookup(baudrates, eeprom->baud_rate), eeprom->baud_rate);
    fprintf(fout, "  Return Delay Time (9):\t%d us\n", eeprom->return_delay_time);
    fprintf(fout, "  Drive Mode (10):\t\t%d\n", eeprom->drive_mode);
    fprintf(fout, "  Operating Mode (11):\t\t%s (%d)\n", str_table_lookup(operating_modes, eeprom->operating_mode),
            eeprom->operating_mode);
    fprintf(fout, "   - Bit 0: Servo Direction:\t%s\n",
            (eeprom->operating_mode & (1 << 0) ? "Normal (CCW+, CW-)" : "Reverse (CW+, CCW-)"));
    fprintf(fout, "   - Bit 2: Profile Config:\t%s\n",
            (eeprom->operating_mode & (1 << 2) ? "Velocity-based" : "Time-based"));
    fprintf(fout, "   - Bit 3: Torque Auto-Enable:\t%s\n", bool_lookup(eeprom->operating_mode & (1 << 3)));
    fprintf(fout, "  Secondary ID (12):\t\t%d\n", eeprom->secondary_id);
    fprintf(fout, "  Protocol (13):\t\tv%d\n", eeprom->protocol_type);
    fprintf(fout, "  Homing Offset (20):\t\t%ld\n", eeprom->homing_offset);
    fprintf(fout, "  Moving Threshold (24):\t%.3f rpm (%ld)\n", eeprom->moving_threshold * 0.229,
            eeprom->moving_threshold);
    fprintf(fout, "  Temperature Limit (31):\t%d C\n", eeprom->temperature_limit);
    fprintf(fout, "  Min Voltage Limit (32):\t%d.%d V\n", eeprom->min_voltage_limit / 10,
            eeprom->min_voltage_limit % 10);
    fprintf(fout, "  Max Voltage Limit (34):\t%d.%d V\n", eeprom->max_voltage_limit / 10,
            eeprom->max_voltage_limit % 10);
    fprintf(fout, "  PWM Limit (36):\t\t%.3f%% (%d)\n", eeprom->pwm_limit * 0.113, eeprom->pwm_limit);
    fprintf(fout, "  Velocity Limit (44):\t\t%.3f rpm (%ld)\n", eeprom->velocity_limit * 0.229, eeprom->velocity_limit);
    fprintf(fout, "  Min Position Limit (48):\t%ld\n", eeprom->min_position_limit);
    fprintf(fout, "  Max Position Limit (49):\t%ld\n", eeprom->max_position_limit);
    fprintf(fout, "  Startup Config (60):\t\t%d\n", eeprom->startup_config);
    fprintf(fout, "   - Bit 0: Startup Torque On:\t%s\n", bool_lookup(eeprom->startup_config & (1 << 0)));
    fprintf(fout, "   - Bit 1: RAM Restore:\t%s\n", bool_lookup(eeprom->startup_config & (1 << 1)));
    fprintf(fout, "-----------------------\n");
    return 0;
}

void dynamixel_canmore_cmds_register(void) {
    debug_remote_cmd_register("dynwrite", "[id] [addr] [u8/s8/u16/s16/u32/s32] [value]",
                              "Writes the data to the requested address\n"
                              "Refer to the Servo Register Map for more information:\n"
                              "\thttps://emanual.robotis.com/docs/en/dxl/x/xl430-w250/\n"
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
