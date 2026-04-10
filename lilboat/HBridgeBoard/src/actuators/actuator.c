#include "actuators/actuator.h"

#include "actuators/async_uart.h"
#include "actuators/hiwonder_driver.h"
#include "actuators/persistence.h"

#include "hardware/pio.h"
#include "titan/debug.h"
#include "titan/logger.h"

#include <math.h>

// #define SERVO_MOVE_TIME_MS 1000
#define SERVO_MAX_DPS 300
#define SERVO_POSTMOVE_DELAY_MS 250
#define MAX_NUM_ERRORS 10
#define SERVO_MAX_TARGET_ERROR 5
#define UNITS_PER_DEGREE 1000 / 240.0f  // Should I move this to hiwonder_driver.h since it's spedific to the servo?
#define DEGREES_PER_SECOND 60 / 0.18f   // Should I move this to hiwonder_driver.h since it's spedific to the servo?

uint8_t discovered_id = 0;

const int32_t ROLLOVER_THRESHOLD = 500;  // 32768
const int32_t FULL_RANGE = 1500;

// Handlers
// dxlact_idle_position_handler_t idle_handler;
// dxlact_move_done_handler_t done_handler;

static void split_uint16(uint val, uint8_t *out_lsb, uint8_t *out_msb) {
    *out_lsb = val & 0xFF;
    *out_msb = (val >> 8) & 0xFF;
}

static void servo_ping_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    // if (!err && rx_packet.target_id == id)
    //     connected = true;
    // else
    //     connected = false;

    servo_t *servo = rx_packet.servo;

    if (err) {
        if (servo->num_errors > MAX_NUM_ERRORS)
            servo->connected = false;
        else
            servo->num_errors++;
    }
    else {
        servo->num_errors = 0;
        servo->connected = true;
        LOG_INFO("Actuator connected!");
    }
}

void servo_ping(servo_t *servo) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    ServoPacket_t ping_packet = make_servo_packet(servo, SERVO_TEMP_READ_CMD, SERVO_TEMP_READ_LEN, param_buf);
    ping_packet.on_read = servo_ping_cb;

    enqueue_packet(ping_packet);
}

static void servo_is_armed_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    servo_t *servo = rx_packet.servo;

    if (err || rx_packet.param_buf[0] != servo->desired_armed_state) {
        if (servo->num_errors < MAX_NUM_ERRORS) {
            servo->num_errors++;
            servo_set_armed(servo, servo->desired_armed_state);
        }
        return;
    }

    if (rx_packet.param_buf[0])
        servo->enabled = true;
    else
        servo->enabled = false;
}

void servo_set_armed(servo_t *servo, bool armed) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = armed;
    ServoPacket_t armed_packet =
        make_servo_packet(servo, SERVO_LOAD_OR_UNLOAD_WRITE_CMD, SERVO_LOAD_OR_UNLOAD_WRITE_LEN, param_buf);

    // Set armed then make sure it was set
    enqueue_packet(armed_packet);
    servo->desired_armed_state = armed;

    ServoPacket_t read_packet =
        make_servo_packet(servo, SERVO_LOAD_OR_UNLOAD_READ_CMD, SERVO_LOAD_OR_UNLOAD_READ_LEN, param_buf);
    read_packet.on_read = servo_is_armed_cb;
    enqueue_packet(read_packet);
}

// Note: a servo struct pointer is passed over user_data
static int64_t servo_move_complete_cb(__unused alarm_id_t id, void *user_data) {
    servo_t *servo = user_data;

    servo->move_active = false;
    servo_read_deg(servo);

    if (servo->return_home_after_move) {
        servo_go_home(servo);
        servo->return_home_after_move = false;
    }

    return false;
}

static void servo_read_target_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    servo_t *servo = rx_packet.servo;

    if (err ||
        abs((rx_packet.param_buf[0] | rx_packet.param_buf[1] << 8) - servo->target_pos) > SERVO_MAX_TARGET_ERROR) {
        // LOG_WARN("Got bad servo target: %hd", rx_packet.param_buf[0] | rx_packet.param_buf[1] << 8);
        // LOG_WARN("Expected %hd", target_pos);
        LOG_WARN("Got bad servo target. Retransmitting...");
        if (servo->num_errors < MAX_NUM_ERRORS) {
            servo->num_errors++;
            servo_set_deg(servo, servo->target_pos * (240.0 / 1000.0));
        }
        else {
            servo->move_active = false;
        }
        return;
    }

    LOG_INFO("Read servo target %hd", rx_packet.param_buf[0] | rx_packet.param_buf[1] << 8);
    servo->move_complete_alarm =
        add_alarm_in_ms(servo->curr_move_time_ms + SERVO_POSTMOVE_DELAY_MS, servo_move_complete_cb, servo, true);
}

static void servo_read_target(servo_t *servo) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    ServoPacket_t read_target_packet =
        make_servo_packet(servo, SERVO_MOVE_TIME_READ_CMD, SERVO_MOVE_TIME_READ_LEN, param_buf);
    read_target_packet.on_read = servo_read_target_cb;

    enqueue_packet(read_target_packet);
}

uint16_t servo_set_deg(servo_t *servo, float deg) {
    uint8_t param_buf[MAX_PACKET_SIZE];

    // float bound_deg = min(max(deg, SERVO_MIN_DEG), SERVO_MAX_DEG);  // TODO: actually do something like this
    uint16_t target = deg * (1000.0 / 240.0);

    uint16_t move_time_ms = fabs(deg - servo->curr_deg) * (1.0 / SERVO_MAX_DPS) * 1000;

    split_uint16(target, &param_buf[0], &param_buf[1]);
    split_uint16(move_time_ms, &param_buf[2], &param_buf[3]);

    LOG_INFO("Set position to %hd in %hd ms", target, move_time_ms);

    ServoPacket_t set_target_packet =
        make_servo_packet(servo, SERVO_MOVE_TIME_WRITE_CMD, SERVO_MOVE_TIME_WRITE_LEN, param_buf);

    servo->target_deg = deg;
    servo->target_pos = target;
    enqueue_packet(set_target_packet);
    servo->move_active = true;
    servo->curr_move_time_ms = move_time_ms;

    servo_read_target(servo);

    return move_time_ms;
}

static void servo_read_deg_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    servo_t *servo = rx_packet.servo;

    // Set off retransmit if error occurred and below error threshold
    if (err) {
        if (servo->num_errors < MAX_NUM_ERRORS) {
            servo->num_errors++;
            servo_read_deg(servo);
        }
        return;
    }

    uint16_t pos = rx_packet.param_buf[0] | rx_packet.param_buf[1] << 8;
    pos = pos > 1000 ? 1000 : pos;
    pos = pos < 0 ? 0 : pos;
    servo->curr_deg = pos * (240.0 / 1000.0);

    LOG_INFO("Read %hd pos as %hd deg", pos, servo->curr_deg);

    if (servo->is_sethome_req) {
        servo->home_deg = servo->curr_deg;
        servo->is_sethome_req = false;
    }
}

void servo_read_deg(servo_t *servo) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    ServoPacket_t read_deg_packet = make_servo_packet(servo, SERVO_POS_READ_CMD, SERVO_POS_READ_LEN, param_buf);
    read_deg_packet.on_read = servo_read_deg_cb;

    enqueue_packet(read_deg_packet);
}

// For now, just big trust this command goes through
static int64_t servo_continuous_stop_cb(__unused alarm_id_t id, void *user_data) {
    servo_t *servo = user_data;

    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = 1;  // Place into motor mode
    split_uint16(0, &param_buf[2], &param_buf[3]);

    ServoPacket_t stop_packet =
        make_servo_packet(servo, SERVO_OR_MOTOR_MODE_WRITE_CMD, SERVO_OR_MOTOR_MODE_WRITE_LEN, param_buf);

    enqueue_packet(stop_packet);

    return 0;
}

// For now, just big trust this command goes through
void servo_continuous_move_ms(servo_t *servo, int16_t speed, uint32_t ms) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = 1;  // Place into motor mode
    split_uint16(speed, &param_buf[2], &param_buf[3]);

    ServoPacket_t continuous_move_packet =
        make_servo_packet(servo, SERVO_OR_MOTOR_MODE_WRITE_CMD, SERVO_OR_MOTOR_MODE_WRITE_LEN, param_buf);

    enqueue_packet(continuous_move_packet);

    add_alarm_in_ms(ms, servo_continuous_stop_cb, servo, true);
}

// For now, just big trust this command goes through
void servo_continuous_set_deg(servo_t *servo, int16_t speed, float deg) {}

void servo_set_absolute_home(servo_t *servo, bool val) {
    servo->is_homing = val;
}

void servo_direct_stop(servo_t *servo) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = 1;
    param_buf[2] = 0x00;
    param_buf[3] = 0x00;

    ServoPacket_t stop_servo_packet =
        make_servo_packet(servo, SERVO_OR_MOTOR_MODE_WRITE_CMD, SERVO_OR_MOTOR_MODE_WRITE_LEN, param_buf);
    enqueue_packet(stop_servo_packet);

    servo->is_moving = false;
}

void servo_stop_check(servo_t *servo) {
    if (!servo->is_moving) {
        return;
    }

    const int32_t STOPPING_TOLERANCE = 300;  // Adjust value

    // printf("Position difference: %d\n", abs(servo->absolute_pos - servo->target_pos_continuous));

    if (abs(servo->absolute_pos - servo->target_pos_continuous) < STOPPING_TOLERANCE) {
        // Stop servo
        uint8_t param_buf[MAX_PACKET_SIZE];
        param_buf[0] = 1;
        param_buf[2] = 0x00;
        param_buf[3] = 0x00;

        ServoPacket_t stop_servo_packet =
            make_servo_packet(servo, SERVO_OR_MOTOR_MODE_WRITE_CMD, SERVO_OR_MOTOR_MODE_WRITE_LEN, param_buf);

        if (enqueue_packet(stop_servo_packet)) {
            servo->is_moving = false;
            servo->flash_write_pending = true;
            servo->still_count = 0;
        }
    }
}

void servo_continuous_move_deg(servo_t *servo, int32_t target_deg, int16_t speed) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = 1;
    param_buf[2] = (uint8_t) (speed & 0x00FF);
    param_buf[3] = (uint8_t) (speed >> 8);

    servo->commanded_speed = speed;

    servo->target_pos_continuous = (servo->absolute_pos + (UNITS_PER_DEGREE * target_deg));

    ServoPacket_t write_continuous_deg_packet =
        make_servo_packet(servo, SERVO_OR_MOTOR_MODE_WRITE_CMD, SERVO_OR_MOTOR_MODE_WRITE_LEN, param_buf);

    if (enqueue_packet(write_continuous_deg_packet)) {
        servo->is_moving = true;
    }
}

bool stall_detected(servo_t *servo) {
    float measured_speed = abs(servo->absolute_pos - servo->position_start_frame) / 0.2f;
    float current_speed_degrees = measured_speed * (1 / UNITS_PER_DEGREE);

    float expected_speed_degrees = (abs(servo->commanded_speed) / 1000.0f) * DEGREES_PER_SECOND;
    float stall_threshold = expected_speed_degrees * 0.10f;

    printf("Expected speed: %f Current speed: %f", expected_speed_degrees, current_speed_degrees);

    if (current_speed_degrees < stall_threshold) {
        return true;
    }

    return false;
}

void servo_read_continuous_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    if (err != SERVO_READ_OK) {
        return;
    }
    servo_t *servo = rx_packet.servo;

    // printf("Servo id: %d", servo_config.servo_info[0].id);

    /*
    if (!servo->is_moving) {
        return;
    }
    */

    // printf("Servo absolute position: %d\n", servo->absolute_pos);

    // int16_t last_position = (int16_t) servo->absolute_pos;

    int16_t last_position = servo->last_position;
    int16_t curr_position = rx_packet.param_buf[1] << 8 | rx_packet.param_buf[0];

    static int16_t ms_counter;

    if (servo->is_homing) {
        servo->absolute_pos = 0;
        servo->last_position = curr_position;

        update_servo_persistent_position(servo, &servo_config);
        write_to_flash(&servo_config);

        servo->is_homing = false;
        return;
    }

    if (servo->needs_sync) {
        servo->last_position = curr_position;

        printf("Servo last position: %d", servo->last_position);

        servo->needs_sync = false;
        return;
    }

    if (servo->flash_write_pending) {  // TODO: Get rid of magic number
        if (abs(curr_position - last_position) <= 1) {
            servo->still_count++;
            if (servo->still_count < 4) {  // TODO: Get rid of magic number
                return;
            }
            if (!update_servo_persistent_position(servo, &servo_config)) {
                LOG_WARN("Error updating servo position");
            }
            write_to_flash(&servo_config);

            servo->flash_write_pending = false;
        }
        else {  // External forces
            servo->still_count = 0;
        }
    }

    if (servo->is_moving) {
        ms_counter += 10;

        if (ms_counter == 200) {  // Change to #define
            if (stall_detected(servo)) {
                // Stop servo
                servo_direct_stop(servo);
            }
            ms_counter = 0;
            servo->position_start_frame = servo->absolute_pos;
        }
    }

    // printf("Absolute positions: %d\n", servo->absolute_pos);

    // printf("Current position: %d", curr_position);

    int32_t position_change = curr_position - last_position;

    if (position_change <= -ROLLOVER_THRESHOLD) {
        servo->absolute_pos += FULL_RANGE;
    }
    else if (position_change >= ROLLOVER_THRESHOLD) {
        servo->absolute_pos -= FULL_RANGE;
    }

    servo->absolute_pos += position_change;
    // printf("Servo absolute position: %d\n", servo->absolute_pos);
    // servo->curr_position = curr_position;
    servo->last_position = curr_position;
    servo_stop_check(servo);

    // printf("Servo absolute position: %d\nServo last position: %d\n", servo->absolute_pos, servo->last_position);
}

void servo_read_continuous(servo_t *servo) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    //  param_buf[0] = 1;
    //  param_buf[1] = 0;

    ServoPacket_t read_continuous_packet = make_servo_packet(servo, SERVO_POS_READ_CMD, SERVO_POS_READ_LEN, param_buf);
    read_continuous_packet.on_read = servo_read_continuous_cb;

    enqueue_packet(read_continuous_packet);
}

void servo_set_id(uint8_t old_id, uint8_t new_id) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = new_id;
    servo_t tmp = { .id = old_id };
    ServoPacket_t set_id_packet = make_servo_packet(&tmp, SERVO_ID_WRITE_CMD, SERVO_ID_WRITE_LEN, param_buf);

    enqueue_packet(set_id_packet);
}

static void servo_read_id_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    // servo *servo = rx_packet.servo;

    if (err != SERVO_READ_OK && err != SERVO_INCORRECT_RESPONDER) {
        // if (servo->num_errors < MAX_NUM_ERRORS) {
        //     servo->num_errors++;
        //     servo_read_id();
        // }
        return;
    }

    discovered_id = rx_packet.param_buf[0];
}

void servo_read_id() {
    uint8_t param_buf[MAX_PACKET_SIZE];
    servo_t tmp = { .id = 0xFE };
    ServoPacket_t read_id_packet = make_servo_packet(&tmp, SERVO_ID_READ_CMD, SERVO_ID_READ_LEN, param_buf);
    read_id_packet.on_read = servo_read_id_cb;

    discovered_id = 0;
    enqueue_packet(read_id_packet);
}

void servo_go_home(servo_t *servo) {
    servo->homed = true;
    servo_set_deg(servo, servo->home_deg);
}

void servo_set_home(servo_t *servo) {
    servo->is_sethome_req = true;
    servo->homed = true;
    servo_read_deg(servo);
}

// static int64_t servo_go_home_cb(__unused alarm_id_t id, __unused void *user_data) {
//     servo_go_home();
//     return false;
// }

void servo_set_deg_then_home(servo_t *servo, float deg) {
    uint16_t move_time_ms = servo_set_deg(servo, deg);
    // add_alarm_in_ms(move_time_ms + SERVO_POSTMOVE_DELAY_MS, servo_go_home_cb, NULL, true);
    servo->return_home_after_move = true;
}

static int parse_int(const char *str, long long *val_out) {
    char *end;
    long long val;
    if (str[0] == '0' && str[1] == 'x') {
        val = strtoll(&str[2], &end, 16);
    }
    else {
        val = strtoll(str, &end, 10);
    }
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

static int debug_set_id_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 3) {
        fprintf(fout, "Incorrect number of arguments (%d)! Needs old and new ID", argc - 1);
        return 1;
    }

    long long old_id = 0, new_id = 0;
    parse_int(argv[1], &old_id);
    parse_int(argv[2], &new_id);

    if (old_id < SERVO_ID_MIN || old_id > SERVO_ID_MAX || new_id < SERVO_ID_MIN || new_id > SERVO_ID_MAX) {
        fprintf(fout, "ID is invalid! It must be between %d and %d", SERVO_ID_MIN, SERVO_ID_MAX);
        return 1;
    }

    servo_set_id(old_id, new_id);
    return 0;
}

static int debug_discover_servo_cb(size_t argc, const char *const *argv, FILE *fout) {
    servo_read_id();

    absolute_time_t packet_wait_time = make_timeout_time_ms(100);
    while (!time_reached(packet_wait_time)) {
        sleep_ms(1);
    }

    if (!discovered_id) {
        fprintf(fout, "Servo not found");
        return 1;
    }

    fprintf(fout, "Discovered servo id %hhu", discovered_id);
    return 0;
}

static int debug_ping_servo(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
        fprintf(fout, "Incorrect number of arguments (%d)! Needs an ID to search for", argc - 1);
        return 1;
    }

    long long id = 0;
    parse_int(argv[1], &id);

    if (id < SERVO_ID_MIN || id > SERVO_ID_MAX) {
        fprintf(fout, "ID is invalid! It must be between %d and %d", SERVO_ID_MIN, SERVO_ID_MAX);
        return 1;
    }

    servo_t tmp = { .id = id };
    servo_ping(&tmp);

    absolute_time_t packet_wait_time = make_timeout_time_ms(100);
    while (!time_reached(packet_wait_time)) {
        sleep_ms(1);
    }

    if (tmp.connected) {
        fprintf(fout, "Servo with ID %lld found", id);
    }
    else {
        fprintf(fout, "Servo not found");
    }

    return 0;
}

void servo_init_internal() {
    gpio_disable_pulls(SERVO_PIN);
    async_uart_init(pio0, 0, SERVO_PIN, UART_BAUD, UART_TIMEOUT_MS);

    debug_remote_cmd_register("hwsetid", "[old_id] [new_id]", "Sets a servo of ID [old_id] to the desired ID [1, 252]",
                              debug_set_id_cb);
    debug_remote_cmd_register(
        "hwfind", "",
        "Discover the ID of a servo on the bus. Only one servo can be connected for this command to function.",
        debug_discover_servo_cb);

    debug_remote_cmd_register("hwping", "[id]", "Check if a servo with [id] is connected", debug_ping_servo);

    // servo_go_home();
}

void make_servo(servo_t *servo, uint8_t id, uint16_t home_deg) {
    servo->is_sethome_req = false;
    servo->return_home_after_move = false;
    servo->desired_armed_state = false;

    servo->id = id;
    servo->home_deg = home_deg;

    printf("Servo id: %d\n", servo->id);
}
