#include "dynamixel_canmore_cmds.h"

#if TITAN_DEBUG_PRESENT
#include "dynamixel_comms.h"
#include "dynamixel_schedule.h"

#include "pico/time.h"
#include "titan/debug.h"

#include <string.h>

// PICO_CONFIG: DYNAMIXEL_CANMORE_RX_TIMEOUT_MS, Timeout to wait for packet response in milliseconds before reporting timeout (must be low to prevent watchdog starvation), min=1, default=30, group=driver_dynamixel
#ifndef DYNAMIXEL_CANMORE_RX_TIMEOUT_MS
#define DYNAMIXEL_CANMORE_RX_TIMEOUT_MS 60
#endif

// ========================================
// CANmore String Utility Functions
// ========================================

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
            return 1;                                                                                                  \
        }                                                                                                              \
        (var_out) = val_tmp;                                                                                           \
    } while (0)

#define parse_u8(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT8_MAX)
#define parse_u16(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT16_MAX)
#define parse_u32(str, var_out) parse_int_with_bounds(str, var_out, 0, UINT32_MAX)
#define parse_i8(str, var_out) parse_int_with_bounds(str, var_out, INT8_MIN, INT8_MAX)
#define parse_i16(str, var_out) parse_int_with_bounds(str, var_out, INT16_MIN, INT16_MAX)
#define parse_i32(str, var_out) parse_int_with_bounds(str, var_out, INT32_MIN, INT32_MAX)

#define str_table_lookup(table, idx) ((idx) < (sizeof(table) / sizeof(*table)) ? table[idx] : "Unknown")
#define bool_lookup(val) ((val) ? "Yes" : "No")

// ========================================
// Dynamixel Request/Response Utility Code
// ========================================

union dynamixel_reg {
    int8_t i8;
    uint8_t u8;
    int16_t i16;
    uint16_t u16;
    int32_t i32;
    uint32_t u32;
    uint8_t bytes[4];  // Max length of any dynamixel packet is 4 bytes
};

struct dynamixel_pending_cmd_state {
    absolute_time_t wait_timeout;
    bool in_progress;
    dynamixel_error_t err;
    const char *cb_err_msg;
    union {
        struct {
            uint16_t model_num;
            uint8_t firmware_ver;
        } ping_pkt;
    } resp;
};
static volatile struct dynamixel_pending_cmd_state pending_cmd_state;

static const char *const async_uart_error_str_lookup[] = { "OK",          "BUSY",      "TIMEOUT", "INCOMPLETE",
                                                           "FRAME_ERROR", "DATA_LOST", "ABORTED" };

static const char *const dynamixel_error_source_lookup[] = { "dynamixel_schedule.c", "dynamixel_comms.c" };

static const char *const dynamixel_resp_err_lookup[] = { "OK",
                                                         "Result Fail",
                                                         "Instruction Error",
                                                         "CRC Error",
                                                         "Data Range Error",
                                                         "Data Length Error",
                                                         "Data Limit Error",
                                                         "Access Error" };

static const char *const dynamixel_dxlret_lookup[] = { "OK",
                                                       "PROCEEDING",
                                                       "ERROR_NOT_SUPPORTED",
                                                       "ERROR_TIMEOUT",
                                                       "ERROR_INVAILD_ID",
                                                       "ERROR_NOT_SUPPORT_BROADCAST",
                                                       "ERROR_NULLPTR",
                                                       "ERROR_LENGTH",
                                                       "ERROR_INVAILD_ADDR",
                                                       "ERROR_ADDR_LENGTH",
                                                       "ERROR_BUFFER_OVERFLOW",
                                                       "ERROR_PORT_NOT_OPEN",
                                                       "ERROR_WRONG_PACKET",
                                                       "ERROR_CHECK_SUM",
                                                       "ERROR_CRC",
                                                       "ERROR_INVAILD_DATA_LENGTH",
                                                       "ERROR_MEMORY_ALLOCATION_FAIL",
                                                       "ERROR_INVAILD_PROTOCOL_VERSION",
                                                       "ERROR_NOT_INITIALIZED",
                                                       "ERROR_NOT_ENOUGH_BUFFER_SIZE",
                                                       "ERROR_PORT_WRITE" };

/**
 * @brief Decodes the requested error and prints it to fout
 *
 * @param fout CANmore Message Output
 * @param error The error to decode
 */
static void decode_dynamixel_error(FILE *fout, dynamixel_error_t error) {
    bool uses_wrapped_code = false;
    fprintf(fout, " - Error:\t");

    switch (error.fields.error) {
    case DYNAMIXEL_ERROR_NONE:
        fprintf(fout, "None (Successful)\n");
        break;
    case DYNAMIXEL_DRIVER_ERROR:
        fprintf(fout, "Internal Driver Error\n");
        if (error.fields.wrapped_error_code != 0) {
            fprintf(fout, " - DXL Lib Err:\t%s\n",
                    str_table_lookup(dynamixel_dxlret_lookup, error.fields.wrapped_error_code));
            uses_wrapped_code = true;
        }
        break;
    case DYNAMIXEL_TTL_ERROR:
        fprintf(fout, "Low-level Async UART Error\n");
        fprintf(fout, " - Wrapped Err:\t%s\n",
                str_table_lookup(async_uart_error_str_lookup, error.fields.wrapped_error_code));
        uses_wrapped_code = true;
        break;
    case DYNAMIXEL_REQUEST_ERROR:
        fprintf(fout, "Request Create\n");
        if (error.fields.wrapped_error_code != 0) {
            fprintf(fout, " - DXL Lib Err:\t%s\n",
                    str_table_lookup(dynamixel_dxlret_lookup, error.fields.wrapped_error_code));
            uses_wrapped_code = true;
        }
        break;
    case DYNAMIXEL_PACKET_ERROR:
        fprintf(fout, "Response Packet Parse Error\n");
        if (error.fields.wrapped_error_code != 0) {
            fprintf(fout, " - DXL Lib Err:\t%s\n",
                    str_table_lookup(dynamixel_dxlret_lookup, error.fields.wrapped_error_code));
            uses_wrapped_code = true;
        }
        break;
    case DYNAMIXEL_RESPONSE_ERROR:
        fprintf(fout, "Error Set in Packet Response\n");
        fprintf(fout, " - Resp Err:\t%s\n",
                str_table_lookup(dynamixel_resp_err_lookup, error.fields.wrapped_error_code));
        uses_wrapped_code = true;
        break;
    case DYNAMIXEL_CMD_QUEUE_FULL_ERROR:
        fprintf(fout, "Command Queue Full\n");
        break;
    case DYNAMIXEL_INVALID_ID:
        fprintf(fout, "Packet received not from intended target\n");
        fprintf(fout, " - ID Found:\t%d\n", error.fields.wrapped_error_code);
        uses_wrapped_code = true;
        break;
    default:
        fprintf(fout, "Unknown (%d)\n", error.fields.error);
        break;
    }

    // If we didn't use the wrapped code, but it's nonzero, print it
    if (!uses_wrapped_code && error.fields.wrapped_error_code != 0) {
        fprintf(fout, " - Wrapped Err:\t%d\n", error.fields.wrapped_error_code);
    }
    // Print where the error occurred
    fprintf(fout, " - Source:\t%s:%d\n", str_table_lookup(dynamixel_error_source_lookup, error.fields.error_source),
            error.fields.line);
}

/**
 * @brief Utility function to queue the requested dynamixel packet, and wait for a response.
 * This is used by all canmore commands which queue a packet
 *
 * @param fout CANmore Message Output
 * @param packet The packet to transmit
 * @param callback Callback for packet
 * @return int
 */
static bool queue_and_wait_for_resp(FILE *fout, InfoToMakeDXLPacket_t *packet, dynamixel_request_cb callback) {
    // Make sure we don't have another comamnd in progress (this only happens if we timed out on the last command)
    if (pending_cmd_state.in_progress) {
        fprintf(fout, "Error: There is already an outstanding packet sent from CANmore that previously timed out\n");
        fprintf(fout, "If it has been over a second since the last request, it is likely a driver bug\n");
        return false;
    }

    // Schedule the packet
    pending_cmd_state.in_progress = true;
    if (!dynamixel_schedule_raw_packet(packet, callback)) {
        fprintf(fout, "Failed to schedule write packet - Transmit Queue Full\n");
        return false;
    }

    // Wait for response
    pending_cmd_state.wait_timeout = make_timeout_time_ms(DYNAMIXEL_CANMORE_RX_TIMEOUT_MS);
    while (pending_cmd_state.in_progress) {
        if (time_reached(pending_cmd_state.wait_timeout)) {
            fprintf(fout, "Error: Did not receive response from driver within timeout\n");
            return false;
        }
        sleep_ms(1);
    }

    // Check if an error was reported by the driver
    if (pending_cmd_state.err.fields.error != DYNAMIXEL_ERROR_NONE) {
        // Print the error
        fprintf(fout, "Error Received from Driver: 0x%08lX\n", pending_cmd_state.err.data);
        decode_dynamixel_error(fout, pending_cmd_state.err);
        return false;
    }

    // Check if an error message was set by the callback
    if (pending_cmd_state.cb_err_msg != NULL) {
        // Print the error
        fprintf(fout, "Error while processing response:\n\t%s\n", pending_cmd_state.cb_err_msg);
        return false;
    }

    return true;
}

// ========================================
// Dynamixel CANmore Comm Commands
// ========================================

static void noresp_dynamixel_pkt_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    (void) result;

    pending_cmd_state.cb_err_msg = NULL;
    pending_cmd_state.err = err;
    if (err.fields.error == DYNAMIXEL_ERROR_NONE) {
        if (result->packet->recv_param_len != 0) {
            pending_cmd_state.cb_err_msg = "Received params in response, when none were expected";
        }
    }

    pending_cmd_state.in_progress = false;
    dynamixel_schedule_next_transfer_or_release();
}

static int dynwrite_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 5) {
        fprintf(fout, "Usage: dynwrite [id] [addr] [u8/s8/u16/s16/u32/s32/bytes] [data...]\n");
        return 1;
    }

    uint8_t id;
    uint16_t addr;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);
    parse_u16(argv[2], addr);

    union dynamixel_reg write_data;
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

    // Borrow code from schedule_write_packet, but instead of firing error callback, report error to console
    size_t max_pkt_len = dynamixel_calc_packet_worst_case_len(dynamixel_write_packet_param_hdr_len + write_len);
    if (max_pkt_len > DYNAMIXEL_PACKET_BUFFER_SIZE) {
        max_pkt_len = DYNAMIXEL_PACKET_BUFFER_SIZE;
    }

    // Create the packet
    InfoToMakeDXLPacket_t packet;
    uint8_t packet_buf[max_pkt_len];
    enum DXLLibErrorCode ret =
        dynamixel_create_write_packet(&packet, packet_buf, sizeof(packet_buf), id, addr, write_data.bytes, write_len);

    // Make sure the packet was created ok
    if (ret != DXL_LIB_OK) {
        fprintf(fout, "Failed to create write packet - Create failed with DXL error code: %d\n", ret);
        return 1;
    }

    // Wait for response
    if (!queue_and_wait_for_resp(fout, &packet, noresp_dynamixel_pkt_cb)) {
        // The function above generates all the errors
        return 1;
    }

    // We don't have anything else to do, any errors are handled by the wait function above
    fprintf(fout, "Wrote %d bytes starting at %d to dynamixel %d\n", write_len, addr, id);
    return 0;
}

static void dynping_dynamixel_pkt_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    (void) result;

    pending_cmd_state.cb_err_msg = NULL;
    pending_cmd_state.err = err;
    if (err.fields.error == DYNAMIXEL_ERROR_NONE) {
        if (result->packet->recv_param_len != 3) {
            pending_cmd_state.cb_err_msg = "Invalid Packet Length Received (Expected 3 bytes)";
        }
        else {
            pending_cmd_state.resp.ping_pkt.model_num =
                result->packet->p_param_buf[0] | (((uint16_t) result->packet->p_param_buf[1]) << 8);
            pending_cmd_state.resp.ping_pkt.firmware_ver = result->packet->p_param_buf[2];
        }
    }

    pending_cmd_state.in_progress = false;
    dynamixel_schedule_next_transfer_or_release();
}

static int dynping_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
        fprintf(fout, "Usage: dynping [id]\n");
        return 1;
    }

    uint8_t id;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);

    // Create the packet
    InfoToMakeDXLPacket_t packet;
    uint8_t packet_buf[dynamixel_calc_packet_worst_case_len(dynamixel_ping_packet_param_len)];
    enum DXLLibErrorCode ret = dynamixel_create_ping_packet(&packet, packet_buf, sizeof(packet_buf), id);

    // Make sure the packet was created ok
    if (ret != DXL_LIB_OK) {
        fprintf(fout, "Failed to create ping packet - Create failed with DXL error code: %d\n", ret);
        return 1;
    }

    // Wait for response
    if (!queue_and_wait_for_resp(fout, &packet, dynping_dynamixel_pkt_cb)) {
        // The function above generates all the errors
        return 1;
    }

    // Got ping back, print info
    uint16_t model_num = pending_cmd_state.resp.ping_pkt.model_num;
    uint8_t fw_ver = pending_cmd_state.resp.ping_pkt.firmware_ver;
    fprintf(fout, "Ping Received from Device %d (Model: %d; Firmware: v%d)\n", id, model_num, fw_ver);
    return 0;
}

static int dynreboot_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
        fprintf(fout, "Usage: dynreboot [id]\n");
        return 1;
    }

    uint8_t id;
    parse_int_with_bounds(argv[1], id, 1, 0xFE);

    // Create the packet
    InfoToMakeDXLPacket_t packet;
    uint8_t packet_buf[dynamixel_calc_packet_worst_case_len(dynamixel_ping_packet_param_len)];
    enum DXLLibErrorCode ret = dynamixel_create_reboot_packet(&packet, packet_buf, sizeof(packet_buf), id);

    // Make sure the packet was created ok
    if (ret != DXL_LIB_OK) {
        fprintf(fout, "Failed to create ping packet - Create failed with DXL error code: %d\n", ret);
        return 1;
    }

    // Wait for response
    if (!queue_and_wait_for_resp(fout, &packet, noresp_dynamixel_pkt_cb)) {
        // The function above generates all the errors
        return 1;
    }

    // We don't have anything else to do, any errors are handled by the wait function above
    fprintf(fout, "Reboot Successful\n");
    return 0;
}

// ========================================
// Dynamixel CANmore Schedule Commands
// ========================================

static const char *const velocity_profiles[] = { "Not Used", "Rectangular", "Triangular", "Trapezoidal" };

static int dynram_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
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
    if (argc != 2) {
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
    fprintf(fout, "   - Bit 0: Servo Direction:\t%s\n",
            (eeprom->drive_mode & (1 << 0) ? "Reverse (CW+, CCW-)" : "Normal (CCW+, CW-)"));
    fprintf(fout, "   - Bit 2: Profile Config:\t%s\n",
            (eeprom->drive_mode & (1 << 2) ? "Time-based" : "Velocity-based"));
    fprintf(fout, "   - Bit 3: Torque Auto-Enable:\t%s\n", bool_lookup(eeprom->drive_mode & (1 << 3)));
    fprintf(fout, "  Operating Mode (11):\t\t%s (%d)\n", str_table_lookup(operating_modes, eeprom->operating_mode),
            eeprom->operating_mode);
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

static int dynstatus_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    (void) argc;
    (void) argv;

    size_t servo_count = 0;
    struct dynamixel_state *state_arr = dynamixel_schedule_get_state_array(&servo_count);
    if (servo_count > 0) {
        for (size_t i = 0; i < servo_count; i++) {
            const char *state_str;
            if (!state_arr[i].first_connect_attempted) {
                state_str = "Not Polled Yet...";
            }
            else if (!state_arr[i].connected) {
                state_str = "Not Connected";
            }
            else if (!state_arr[i].alert_notified) {
                state_str = "Connected";
            }
            else {
                state_str = "Connected, Alert Present";
            }
            fprintf(fout, "Dynamixel %d: %s\n", state_arr[i].id, state_str);
        }
    }
    else {
        fprintf(fout, "No Dynamixels Configured\n");
    }

    return 0;
}

void dynamixel_canmore_cmds_register(void) {
    debug_remote_cmd_register(
        "dynwrite", "[id] [addr] [reg type] [value]",
        "Writes the data to the requested address\n"
        "The reg type field can be one of [u8/s8/u16/s16/u32/s32]\n"
        "\tThe u/s stands for unsigned/sign (if the variable accepts negative numbers)\n"
        "\tThe 8/16/32 determines the number of bits that the integer takes up (multiply the size in bytes by 8)\n"
        "Refer to the Servo Register Map for more information:\n"
        "\thttps://emanual.robotis.com/docs/en/dxl/x/xl430-w250/\n"
        "Note: All numbers are in decimal\n"
        "Example: dynwrite 4 7 u8 5\n"
        "\tWrites ID register (address 7, 1 byte unsigned) of dynamixel 4, changing its ID to 5",
        dynwrite_cmd_cb);
    debug_remote_cmd_register("dynping", "[id]", "Attempts to ping the requested dynamixel", dynping_cmd_cb);
    debug_remote_cmd_register("dynreboot", "[id]", "Reboots the requested dynamixel", dynreboot_cmd_cb);
    debug_remote_cmd_register("dynram", "[id]",
                              "Dumps the current RAM state of the requested dynamixel\n"
                              "Note this can only read dynamixels that are registered for monitoring the application",
                              dynram_cmd_cb);
    debug_remote_cmd_register("dyneeprom", "[id]",
                              "Dumps the EEPROM of the requested dynamixel\n"
                              "Note this can only read dynamixels that are registered for monitoring the application",
                              dyneeprom_cmd_cb);
    debug_remote_cmd_register("dynstatus", "", "Lists the IDs of all monitored dynamixel servos.", dynstatus_cmd_cb);
}

#else

void dynamixel_canmore_cmds_register(void) {}

#endif
