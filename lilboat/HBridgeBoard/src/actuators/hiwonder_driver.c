#include "actuators/hiwonder_driver.h"

#include "actuators/async_uart.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/u_int8_multi_array.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "titan/logger.h"
#include "titan/queue.h"
#include "titan/version.h"

#include <string.h>

const size_t response_len[SERVO_LED_ERROR_READ_CMD + 1] = {  // Easily access expected read lengths from command
    [SERVO_MOVE_TIME_READ_CMD] = SERVO_MOVE_TIME_READ_RESPONSE_LEN,
    [SERVO_MOVE_TIME_WAIT_READ_CMD] = SERVO_MOVE_TIME_WAIT_READ_RESPONSE_LEN,
    [SERVO_ID_READ_CMD] = SERVO_ID_READ_RESPONSE_LEN,
    [SERVO_ANGLE_OFFSET_READ_CMD] = SERVO_ANGLE_OFFSET_READ_RESPONSE_LEN,
    [SERVO_ANGLE_LIMIT_READ_CMD] = SERVO_ANGLE_LIMIT_READ_RESPONSE_LEN,
    [SERVO_VIN_LIMIT_READ_CMD] = SERVO_VIN_LIMIT_READ_RESPONSE_LEN,
    [SERVO_TEMP_MAX_LIMIT_READ_CMD] = SERVO_TEMP_MAX_LIMIT_READ_RESPONSE_LEN,
    [SERVO_TEMP_READ_CMD] = SERVO_TEMP_READ_RESPONSE_LEN,
    [SERVO_VIN_READ_CMD] = SERVO_VIN_READ_RESPONSE_LEN,
    [SERVO_POS_READ_CMD] = SERVO_POS_READ_RESPONSE_LEN,
    [SERVO_OR_MOTOR_MODE_READ_CMD] = SERVO_OR_MOTOR_MODE_READ_RESPONSE_LEN,
    [SERVO_LOAD_OR_UNLOAD_READ_CMD] = SERVO_LOAD_OR_UNLOAD_READ_RESPONSE_LEN,
    [SERVO_LED_CTRL_READ_CMD] = SERVO_LED_CTRL_READ_RESPONSE_LEN,
    [SERVO_LED_ERROR_READ_CMD] = SERVO_LED_ERROR_READ_RESPONSE_LEN
};
static struct QUEUE_DEFINE(struct ServoPacket, 10) tx_queue = { 0 };

bool packet_in_flight = false;
ServoPacket_t most_recent_sent;
uint8_t raw_rx_packet[MAX_PACKET_SIZE];

uint8_t raw_packet[MAX_PACKET_SIZE];

ServoPacket_t make_servo_packet(servo_t *servo, uint8_t command, uint8_t command_length,
                                uint8_t param_buf[PARAMETER_MTU]) {
    ServoPacket_t tmp = { .servo = servo, .command_length = command_length, .command = command, .on_read = NULL };
    memcpy(tmp.param_buf, param_buf, PARAMETER_MTU);

    return tmp;
}

static uint8_t calculate_checksum(ServoPacket_t *packet) {
    uint data_sum = packet->servo->id + packet->command_length + packet->command;
    for (uint8_t i = 0; i < packet->command_length - 3; i++) {
        data_sum += packet->param_buf[i];
    }

    return (uint8_t) (~data_sum);
}

static void on_packet_received(__unused enum async_uart_rx_err error, uint8_t *raw_packet, __unused size_t len) {
    // Any operatiosn on raw_packet are invalid if error is set, so check that first
    if (error != ASYNC_UART_RX_OK) {
        LOG_ERROR("Async UART reported RX error: %u\n", error);
        ServoPacket_t dummy_packet;
        most_recent_sent.on_read(dummy_packet, SERVO_INTERNAL_UART_ERROR);
        packet_in_flight = false;
        return;
    }

    uint8_t id = raw_packet[2];
    ServoPacket_t rx_packet = { .servo = most_recent_sent.servo,
                                .command_length = raw_packet[3],
                                .command = raw_packet[4] };
    rx_packet.checksum = raw_packet[rx_packet.command_length + HEADER_SIZE + CHECKSUM_SIZE - 1];

    for (uint8_t i = 0; i < rx_packet.command_length - 3; i++) {
        rx_packet.param_buf[i] = raw_packet[i + 5];
    }

    enum servo_read_err err = SERVO_READ_OK;

    if (rx_packet.command != most_recent_sent.command) {
        LOG_ERROR("Received unexpected response type: %hhx. Expected %hhx\n", rx_packet.command,
                  most_recent_sent.command);
        err = SERVO_BAD_RESPONSE_TYPE;
    }
    else if (rx_packet.command_length != response_len[most_recent_sent.command]) {
        LOG_ERROR("Received incorrect response size: %hhu. Expected %hhx from %hhx\n", rx_packet.command_length,
                  response_len[most_recent_sent.command], most_recent_sent.command);
        err = SERVO_BAD_RESPONSE_LENGTH;
    }
    else if (calculate_checksum(&rx_packet) != rx_packet.checksum) {
        LOG_ERROR("Received checksum (%x) didn't match expected (%x)\n", rx_packet.checksum,
                  calculate_checksum(&rx_packet));
        err = SERVO_BAD_CHECKSUM;
    }
    else if (id != most_recent_sent.servo->id) {
        LOG_WARN("Response packet source (%x) didn't match target (%x)\n", id, most_recent_sent.servo->id);
        // This can happen in some commands, so don't return early
        err = SERVO_INCORRECT_RESPONDER;
    }

    most_recent_sent.on_read(rx_packet, err);
    packet_in_flight = false;
}

static void on_packet_sent(__unused enum async_uart_tx_err error) {
    if (most_recent_sent.on_read) {
        memset(raw_rx_packet, 0, MAX_PACKET_SIZE * sizeof(uint8_t));
        async_uart_read(raw_rx_packet, response_len[most_recent_sent.command] + HEADER_SIZE + CHECKSUM_SIZE,
                        on_packet_received);
    }
    else {
        packet_in_flight = false;
    }
}

void send_packet(ServoPacket_t packet) {
    packet_in_flight = true;

    uint packet_size = packet.command_length + HEADER_SIZE + CHECKSUM_SIZE;

    raw_packet[0] = raw_packet[1] = HEADER_CODE;
    raw_packet[2] = packet.servo->id;
    raw_packet[3] = packet.command_length;
    raw_packet[4] = packet.command;

    // TODO: replace with memcpy?
    for (uint8_t i = 0; i < 7 - 3; i++) {
        raw_packet[i + 5] = packet.param_buf[i];
    }

    raw_packet[packet_size - 1] = calculate_checksum(&packet);

    most_recent_sent = packet;
    async_uart_write(raw_packet, packet_size, false, on_packet_sent);
}

bool enqueue_packet(ServoPacket_t packet) {
    if (QUEUE_FULL(&tx_queue)) {
        return false;
    }

    ServoPacket_t *entry = QUEUE_CUR_WRITE_ENTRY(&tx_queue);
    // This copy is generally safe since there are no (non-function) pointers in ServoPacket
    *entry = packet;
    QUEUE_MARK_WRITE_DONE(&tx_queue);

    return true;
}

static bool dequeue_packet(ServoPacket_t *packet) {
    // LOG_INFO("Requested packet dequeue");

    if (QUEUE_EMPTY(&tx_queue))
        return false;

    ServoPacket_t *entry = QUEUE_CUR_READ_ENTRY(&tx_queue);
    *packet = *entry;  // shallow copy
    QUEUE_MARK_READ_DONE(&tx_queue);

    return true;
}

bool uart_scheduler(__unused repeating_timer_t *rt) {
    ServoPacket_t packet;
    // Short-circuit to not dequeue when bus is busy
    if (packet_in_flight || !dequeue_packet(&packet))
        return true;

    uint8_t prev_interrupts = save_and_disable_interrupts();
    send_packet(packet);
    restore_interrupts(prev_interrupts);
    return true;
}
