#include "async_uart.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <stdio.h>

// Taken from Hiwonder datasheet
#define SERVO_MOVE_TIME_WRITE_CMD 1
#define SERVO_MOVE_TIME_READ_CMD 2
#define SERVO_MOVE_TIME_WAIT_WRITE_CMD 7
#define SERVO_MOVE_TIME_WAIT_READ_CMD 8
#define SERVO_MOVE_START_CMD 11
#define SERVO_MOVE_STOP_CMD 12
#define SERVO_ID_WRITE_CMD 13
#define SERVO_ID_READ_CMD 14
#define SERVO_ANGLE_OFFSET_ADJUST_CMD 17
#define SERVO_ANGLE_OFFSET_WRITE_CMD 18
#define SERVO_ANGLE_OFFSET_READ_CMD 19
#define SERVO_ANGLE_LIMIT_WRITE_CMD 20
#define SERVO_ANGLE_LIMIT_READ_CMD 21
#define SERVO_VIN_LIMIT_WRITE_CMD 22
#define SERVO_VIN_LIMIT_READ_CMD 23
#define SERVO_TEMP_MAX_LIMIT_WRITE_CMD 24
#define SERVO_TEMP_MAX_LIMIT_READ_CMD 25
#define SERVO_TEMP_READ_CMD 26
#define SERVO_VIN_READ_CMD 27
#define SERVO_POS_READ_CMD 28
#define SERVO_OR_MOTOR_MODE_WRITE_CMD 29
#define SERVO_OR_MOTOR_MODE_READ_CMD 30
#define SERVO_LOAD_OR_UNLOAD_WRITE_CMD 31
#define SERVO_LOAD_OR_UNLOAD_READ_CMD 32
#define SERVO_LED_CTRL_WRITE_CMD 33
#define SERVO_LED_CTRL_READ_CMD 34
#define SERVO_LED_ERROR_WRITE_CMD 35
#define SERVO_LED_ERROR_READ_CMD 36

#define SERVO_MOVE_TIME_WRITE_LEN 7
#define SERVO_MOVE_TIME_READ_LEN 3
#define SERVO_MOVE_TIME_WAIT_WRITE_LEN 7
#define SERVO_MOVE_TIME_WAIT_READ_LEN 3
#define SERVO_MOVE_START_LEN 3
#define SERVO_MOVE_STOP_LEN 3
#define SERVO_ID_WRITE_LEN 4
#define SERVO_ID_READ_LEN 3
#define SERVO_ANGLE_OFFSET_ADJUST_LEN 4
#define SERVO_ANGLE_OFFSET_WRITE_LEN 3
#define SERVO_ANGLE_OFFSET_READ_LEN 3
#define SERVO_ANGLE_LIMIT_WRITE_LEN 7
#define SERVO_ANGLE_LIMIT_READ_LEN 3
#define SERVO_VIN_LIMIT_WRITE_LEN 7
#define SERVO_VIN_LIMIT_READ_LEN 3
#define SERVO_TEMP_MAX_LIMIT_WRITE_LEN 4
#define SERVO_TEMP_MAX_LIMIT_READ_LEN 3
#define SERVO_TEMP_READ_LEN 3
#define SERVO_VIN_READ_LEN 3
#define SERVO_POS_READ_LEN 3
#define SERVO_OR_MOTOR_MODE_WRITE_LEN 7
#define SERVO_OR_MOTOR_MODE_READ_LEN 3
#define SERVO_LOAD_OR_UNLOAD_WRITE_LEN 4
#define SERVO_LOAD_OR_UNLOAD_READ_LEN 3
#define SERVO_LED_CTRL_WRITE_LEN 4
#define SERVO_LED_CTRL_READ_LEN 3
#define SERVO_LED_ERROR_WRITE_LEN 4
#define SERVO_LED_ERROR_READ_LEN 3

#define UART_PIN 8u
#define UART_BAUD 115200u
#define UART_TIMEOUT_MS 50

#define PARAMETER_MTU 4
#define HEADER_SIZE 2
#define HEADER_CODE 0x55
#define CHECKSUM_SIZE 1

#define MAX_PACKET_SIZE 10

#define SET_TARGET_PIN 2
#define READ_POS_PIN 3
#define GO_HOME_PIN 4
#define ARM_PIN 5
#define READ_ARMED_PIN 6
#define SET_ID_PIN 7
#define READ_ID_PIN 8

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

typedef struct UartPacket {
    uint8_t target_id;
    uint8_t command_length;
    uint8_t command;
    uint8_t param_buf[PARAMETER_MTU];
    uint8_t checksum;
} UartPacket_t;

static UartPacket_t packet;
uint8_t raw_packet[MAX_PACKET_SIZE];
static bool is_read_request = false;

uint8_t calculate_checksum(UartPacket_t *packet) {
    uint data_sum = packet->target_id + packet->command_length + packet->command;
    for (uint8_t i = 0; i < packet->command_length - 3; i++) {
        data_sum += packet->param_buf[i];
    }

    return (uint8_t) (~data_sum);
}

// This function is terrifyingly unsafe.
// TODO: rewrite this better since you actually get the data array in (duh)
static void on_packet_received(enum async_uart_rx_err error, uint8_t *data, size_t len) {
    UartPacket_t rx_packet = { .target_id = raw_packet[2], .command_length = raw_packet[3], .command = raw_packet[4] };

    // Do at least *some* error checking before trying to read into the void
    if (rx_packet.command_length + 3 > MAX_PACKET_SIZE || rx_packet.command_length < 3) {
        LOG_ERROR("Received an improperly-sized packet: length %u\n", rx_packet.command_length + 3);
        return;
    }

    for (uint8_t i = 0; i < rx_packet.command_length - 3; i++) {
        rx_packet.param_buf[i] = raw_packet[i + 5];
    }
    rx_packet.checksum = raw_packet[rx_packet.command_length + HEADER_SIZE + CHECKSUM_SIZE - 1];

    // Make sure everything matches
    if (rx_packet.target_id != packet.target_id) {
        LOG_WARN("Response packet source (%x) didn't match target (%x)\n", rx_packet.target_id, packet.target_id);
        // This can happen in some commands, so don't return early
    }
    else if (calculate_checksum(&rx_packet) != rx_packet.checksum) {
        LOG_ERROR("Received checksum (%x) didn't match expected (%x\n", rx_packet.checksum,
                  calculate_checksum(&rx_packet));
        return;  // This should set off a retransmit either here or in higher logic
    }
    // Someway somehow check if the command value is correct

    // Packet received correctly, make it available
    packet = rx_packet;

    LOG_INFO("Packet data: ");
    for (uint8_t i = 0; i < PARAMETER_MTU; i++)
        LOG_INFO("%x ", packet.param_buf[i]);
    LOG_INFO("\n");
}

static void on_packet_sent(enum async_uart_tx_err error) {
    if (is_read_request) {
        is_read_request = false;
        async_uart_read(raw_packet, packet.command_length + HEADER_SIZE + CHECKSUM_SIZE, on_packet_received);
    }
}

void send_packet() {
    uint packet_size = packet.command_length + HEADER_SIZE + CHECKSUM_SIZE;

    raw_packet[0] = raw_packet[1] = HEADER_CODE;
    raw_packet[2] = packet.target_id;
    raw_packet[3] = packet.command_length;
    raw_packet[4] = packet.command;

    for (uint8_t i = 0; i < packet.command_length - 3; i++) {
        raw_packet[i + 5] = packet.param_buf[i];
    }

    raw_packet[packet_size - 1] = calculate_checksum(&packet);

    async_uart_write(raw_packet, packet_size, false, on_packet_sent);
}

void split_uint(uint val, uint8_t *out_lsb, uint8_t *out_msb) {
    *out_lsb = val & 0xFF;
    *out_msb = (val >> 8) & 0xFF;
}

//
// Start command functions
//

void servo_set_target(uint8_t target, uint angle, uint time) {
    packet.target_id = target;
    packet.command = SERVO_MOVE_TIME_WRITE_CMD;
    packet.command_length = SERVO_MOVE_TIME_WRITE_LEN;
    split_uint(angle, &packet.param_buf[0], &packet.param_buf[1]);
    split_uint(time, &packet.param_buf[2], &packet.param_buf[3]);

    LOG_INFO("Sending move request: %x to %udeg in %us\n", target, angle, time);
    send_packet();
}

void servo_read_pos(uint8_t target) {
    packet.target_id = target;
    packet.command = SERVO_POS_READ_CMD;
    packet.command_length = SERVO_POS_READ_LEN;

    is_read_request = true;
    LOG_INFO("Sending pos read request to %x\n", target);
    send_packet();
}

void servo_set_arm_state(uint8_t target, bool state) {
    packet.target_id = target;
    packet.command = SERVO_LOAD_OR_UNLOAD_WRITE_CMD;
    packet.command_length = SERVO_LOAD_OR_UNLOAD_WRITE_LEN;
    packet.param_buf[0] = (uint8_t) state;

    LOG_INFO("Sending arm request to %x\n", target);
    send_packet();
}

void servo_read_armed(uint8_t target) {
    packet.target_id = target;
    packet.command = SERVO_LOAD_OR_UNLOAD_READ_CMD;
    packet.command_length = SERVO_LOAD_OR_UNLOAD_READ_LEN;

    is_read_request = true;
    LOG_INFO("Sending read arm state request to %x\n", target);
    send_packet();
}

void servo_set_id(uint8_t old_target, uint8_t new_target) {
    packet.target_id = old_target;
    packet.command = SERVO_ID_WRITE_CMD;
    packet.command_length = SERVO_ID_WRITE_LEN;
    packet.param_buf[0] = new_target;

    LOG_INFO("Changing id %x to %x", old_target, new_target);
}

void servo_ping_all() {
    packet.target_id = 254;
    packet.command = SERVO_ID_READ_CMD;
    packet.command_length = SERVO_ID_READ_LEN;

    is_read_request = true;
    LOG_INFO("Requesting bus-wide servo ping\n");
    send_packet();
}

//
// Start pin config and main
//

void configure_pin(uint pin_num) {
    gpio_init(pin_num);
    gpio_set_dir(pin_num, GPIO_IN);
    gpio_set_pulls(pin_num, true, false);
}

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("%s\n", FULL_BUILD_TAG);

    configure_pin(SET_TARGET_PIN);
    configure_pin(READ_POS_PIN);
    configure_pin(GO_HOME_PIN);
    configure_pin(ARM_PIN);
    configure_pin(READ_ARMED_PIN);
    configure_pin(SET_ID_PIN);
    configure_pin(READ_ID_PIN);

    bool value = true;

    async_uart_init(pio0, 0, UART_PIN, UART_BAUD, UART_TIMEOUT_MS);

    while (true) {
        watchdog_update();
        printf("Hello World!\n");

        gpio_put(LED_PIN, value);
        value = !value;

        if (!gpio_get(SET_TARGET_PIN))
            servo_set_target(0x01, 1000, 2000);
        else if (!gpio_get(READ_POS_PIN))
            servo_read_pos(0x01);
        else if (!gpio_get(GO_HOME_PIN))
            servo_set_target(0x01, 0, 2000);
        else if (!gpio_get(ARM_PIN))
            servo_set_arm_state(0x01, true);
        else if (!gpio_get(READ_ARMED_PIN))
            servo_read_armed(0x01);
        else if (!gpio_get(SET_ID_PIN))
            servo_set_id(0x01, 0x02);
        else if (!gpio_get(READ_ID_PIN))
            servo_ping_all();

        sleep_ms(1000);
    }
    return 0;
}
