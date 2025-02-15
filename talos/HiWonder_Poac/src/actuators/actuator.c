#include "actuators/actuator.h"

#include "actuators/async_uart.h"
#include "actuators/hiwonder_driver.h"

#include "hardware/pio.h"

#include <math.h>

// #define SERVO_MOVE_TIME_MS 1000
#define SERVO_MAX_DPS 300
#define SERVO_POSTMOVE_DELAY_MS 250
#define MAX_NUM_ERRORS 5

uint8_t id = 1;
uint32_t max_move_time_ms;

uint16_t home_deg = 100;
uint16_t curr_deg = 0;

// TODO: this is cursed
bool is_sethome_req = false;
// bool return_home_after_move = false;
bool desired_armed_state = false;

// Handlers
// dxlact_idle_position_handler_t idle_handler;
// dxlact_move_done_handler_t done_handler;

// Internal state tracking, safe to modify in interrupts
volatile bool connected;
volatile bool move_active;
volatile bool enabled;
volatile bool homed;
volatile bool hardware_err;
volatile uint8_t num_errors;

// Only valid when move_active is true
int16_t target_position;
absolute_time_t move_timeout;

// Alarms and timers
alarm_id_t move_complete_alarm;
alarm_id_t start_go_home_alarm;

static void split_uint16(uint val, uint8_t *out_lsb, uint8_t *out_msb) {
    *out_lsb = val & 0xFF;
    *out_msb = (val >> 8) & 0xFF;
}

static void servo_ping_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    // if (!err && rx_packet.target_id == id)
    //     connected = true;
    // else
    //     connected = false;

    if (err) {
        if (num_errors > MAX_NUM_ERRORS)
            connected = false;
        else
            num_errors++;
    }
    else {
        num_errors = 0;
        connected = true;
        // LOG_INFO("Actuator connected!");
    }
}

void servo_ping() {
    uint8_t param_buf[MAX_PACKET_SIZE];
    ServoPacket_t ping_packet = make_servo_packet(id, SERVO_TEMP_READ_CMD, SERVO_TEMP_READ_LEN, param_buf);
    ping_packet.on_read = servo_ping_cb;

    enqueue_packet(ping_packet);
}

static void servo_is_armed_cb(ServoPacket_t rx_packet, __unused enum servo_read_err err) {
    if (err || rx_packet.param_buf[0] != desired_armed_state) {
        if (num_errors < MAX_NUM_ERRORS) {
            num_errors++;
            servo_set_armed(desired_armed_state);
        }
        return;
    }

    if (rx_packet.param_buf[0])
        enabled = true;
    else
        enabled = false;
}

void servo_set_armed(bool armed) {
    uint8_t param_buf[MAX_PACKET_SIZE];
    param_buf[0] = armed;
    ServoPacket_t armed_packet =
        make_servo_packet(id, SERVO_LOAD_OR_UNLOAD_WRITE_CMD, SERVO_LOAD_OR_UNLOAD_WRITE_LEN, param_buf);

    // Set armed then make sure it was set
    enqueue_packet(armed_packet);
    desired_armed_state = armed;

    ServoPacket_t read_packet =
        make_servo_packet(id, SERVO_LOAD_OR_UNLOAD_READ_CMD, SERVO_LOAD_OR_UNLOAD_READ_LEN, param_buf);
    read_packet.on_read = servo_is_armed_cb;

    enqueue_packet(read_packet);
}

static int64_t servo_move_complete_cb(__unused alarm_id_t id, __unused void *user_data) {
    move_active = false;
    servo_read_deg();

    // if (return_home_after_move) {
    //     servo_go_home();
    //     return_home_after_move = false;
    // }

    return false;
}

uint16_t servo_set_deg(float deg) {
    uint8_t param_buf[MAX_PACKET_SIZE];

    // float bound_deg = min(max(deg, SERVO_MIN_DEG), SERVO_MAX_DEG);  // TODO: use this
    uint16_t target = deg * (1000.0 / 240.0);
    uint16_t move_time_ms = fabs(deg - curr_deg) * (1.0 / SERVO_MAX_DPS) * 1000;

    split_uint16(target, &param_buf[0], &param_buf[1]);
    split_uint16(move_time_ms, &param_buf[2], &param_buf[3]);  // TODO: calculate time based on max rotation speed

    LOG_INFO("Set position to %hd in %hd ms", target, move_time_ms);

    ServoPacket_t set_target_packet =
        make_servo_packet(id, SERVO_MOVE_TIME_WRITE_CMD, SERVO_MOVE_TIME_WRITE_LEN, param_buf);

    target_position = deg;
    enqueue_packet(set_target_packet);
    move_active = true;
    move_complete_alarm = add_alarm_in_ms(move_time_ms, servo_move_complete_cb, NULL, true);

    return move_time_ms;
}

static void servo_read_deg_cb(ServoPacket_t rx_packet, __unused enum servo_read_err err) {
    // Set off retransmit if error occurred and below error threshold
    if (err) {
        if (num_errors < MAX_NUM_ERRORS) {
            num_errors++;
            servo_read_deg();
        }
        return;
    }

    uint16_t pos = rx_packet.param_buf[0] | rx_packet.param_buf[1] << 8;
    curr_deg = pos * (240.0 / 1000.0);

    LOG_INFO("Read %hd pos as %hd deg", pos, curr_deg);

    if (is_sethome_req) {
        home_deg = curr_deg;
        is_sethome_req = false;
    }
}

void servo_read_deg() {
    uint8_t param_buf[MAX_PACKET_SIZE];
    ServoPacket_t read_deg_packet = make_servo_packet(id, SERVO_POS_READ_CMD, SERVO_POS_READ_LEN, param_buf);
    read_deg_packet.on_read = servo_read_deg_cb;

    enqueue_packet(read_deg_packet);
}

void servo_go_home() {
    homed = true;
    servo_set_deg(home_deg);
}

void servo_set_home() {
    is_sethome_req = true;
    homed = true;
    servo_read_deg();
}

static int64_t servo_go_home_cb(__unused alarm_id_t id, __unused void *user_data) {
    servo_go_home();
    return false;
}

void servo_set_deg_then_home(float deg) {
    uint16_t move_time_ms = servo_set_deg(deg);
    add_alarm_in_ms(move_time_ms + SERVO_POSTMOVE_DELAY_MS, servo_go_home_cb, NULL, true);
    // return_home_after_move = true;
}

void init_servo() {
    gpio_disable_pulls(CLAW_CHECK_PIN);
    async_uart_init(pio0, 0, CLAW_CHECK_PIN, UART_BAUD, UART_TIMEOUT_MS);

    servo_go_home();
}
