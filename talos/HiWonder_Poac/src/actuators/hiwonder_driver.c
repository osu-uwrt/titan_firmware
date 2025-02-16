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

ServoPacket_t make_servo_packet(uint8_t target_id, uint8_t command, uint8_t command_length,
                                uint8_t param_buf[PARAMETER_MTU]) {
    ServoPacket_t tmp = {
        .target_id = target_id, .command_length = command_length, .command = command, .on_read = NULL
    };
    memcpy(tmp.param_buf, param_buf, PARAMETER_MTU);

    return tmp;
}

static uint8_t calculate_checksum(ServoPacket_t *packet) {
    uint data_sum = packet->target_id + packet->command_length + packet->command;
    for (uint8_t i = 0; i < packet->command_length - 3; i++) {
        data_sum += packet->param_buf[i];
    }

    return (uint8_t) (~data_sum);
}

static void on_packet_received(__unused enum async_uart_rx_err error, uint8_t *raw_packet, __unused size_t len) {
    // LOG_INFO("Header? %hhx", raw_rx_packet[0]);
    LOG_INFO("Header: %hhx, %hhx", raw_rx_packet[0], raw_rx_packet[1]);
    LOG_INFO("ID: %hhx", raw_rx_packet[2]);
    LOG_INFO("Command: %hhx", raw_rx_packet[4]);
    LOG_INFO("Len: %hhx", raw_rx_packet[3]);
    LOG_INFO("Checksum: %hhx", raw_rx_packet[len - 1]);

    // Any operatiosn on raw_packet are invalid if error is set, so check that first
    if (error != ASYNC_UART_RX_OK) {
        LOG_ERROR("Async UART reported RX error: %u\n", error);
        ServoPacket_t dummy_packet;
        most_recent_sent.on_read(dummy_packet, SERVO_INTERNAL_UART_ERROR);
        packet_in_flight = false;
        return;
    }

    ServoPacket_t rx_packet = { .target_id = raw_packet[2], .command_length = raw_packet[3], .command = raw_packet[4] };
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
    else if (rx_packet.target_id != most_recent_sent.target_id) {
        LOG_WARN("Response packet source (%x) didn't match target (%x)\n", rx_packet.target_id,
                 most_recent_sent.target_id);
        // This can happen in some commands, so don't return early
        err = SERVO_INCORRECT_RESPONDER;
    }

    // LOG_INFO("Receieved response packet");
    // for (int i = 0; i < len; i++) {
    //     LOG_INFO("%hhx", raw_packet[i]);
    // }

    most_recent_sent.on_read(rx_packet, err);
    packet_in_flight = false;

    // LOG_INFO("Packet receive complete; marking packet_in_flight as false");

    // LOG_INFO("Packet data: ");
    // for (uint8_t i = 0; i < PARAMETER_MTU; i++)
    //     LOG_INFO("%x ", rx_packet.param_buf[i]);

    // LOG_INFO("Raw packet: ");
    // for (uint8_t i = 0; i < rx_packet.command_length + HEADER_SIZE + CHECKSUM_SIZE; i++)
    //     LOG_INFO("%x ", raw_packet[i]);
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

    // LOG_INFO("Packet sent callback");
}

void send_packet(ServoPacket_t packet) {
    packet_in_flight = true;

    uint packet_size = packet.command_length + HEADER_SIZE + CHECKSUM_SIZE;

    raw_packet[0] = raw_packet[1] = HEADER_CODE;
    raw_packet[2] = packet.target_id;
    raw_packet[3] = packet.command_length;
    raw_packet[4] = packet.command;
    // raw_packet[2] = 1;
    // raw_packet[3] = SERVO_MOVE_TIME_WRITE_CMD;
    // raw_packet[4] = SERVO_MOVE_TIME_WRITE_LEN;

    // TODO: replace with memcpy?
    for (uint8_t i = 0; i < 7 - 3; i++) {
        raw_packet[i + 5] = packet.param_buf[i];
        // raw_packet[i + 5] = i;
    }

    raw_packet[packet_size - 1] = calculate_checksum(&packet);

    // uint8_t prev_interrupts = save_and_disable_interrupts();
    most_recent_sent = packet;
    // LOG_INFO("Writing pacekt to UART line; packet_in_flight marked true");

    // LOG_INFO("Header: %hhx", raw_packet[0]);
    // LOG_INFO("ID: %hhx", raw_packet[2]);
    // LOG_INFO("Command: %hhx", raw_packet[4]);
    // LOG_INFO("Len: %hhx", raw_packet[3]);
    // LOG_INFO("Checksum: %hhx", raw_packet[packet_size - 1]);

    // for (int i = 0; i < packet_size; i++) {
    //     LOG_INFO("%hhx", raw_packet[i]);
    // }
    async_uart_write(raw_packet, packet_size, false, on_packet_sent);
    // restore_interrupts(prev_interrupts);
}

bool enqueue_packet(ServoPacket_t packet) {
    if (QUEUE_FULL(&tx_queue))
        return false;

    ServoPacket_t *entry = QUEUE_CUR_WRITE_ENTRY(&tx_queue);
    // This copy is generally safe since there are no (non-function) pointers in ServoPacket
    *entry = packet;
    // entry->target_id = packet.target_id;
    // entry->command = packet.command;
    // entry->command_length = packet.command_length;
    // memcpy(entry->param_buf, packet.param_buf, sizeof(packet.param_buf));
    // entry->on_read = packet.on_read;
    QUEUE_MARK_WRITE_DONE(&tx_queue);

    return true;
}

static bool dequeue_packet(ServoPacket_t *packet) {
    // LOG_INFO("Requested packet dequeue");

    if (QUEUE_EMPTY(&tx_queue))
        return false;

    // LOG_INFO("Dequeue request successful");

    ServoPacket_t *entry = QUEUE_CUR_READ_ENTRY(&tx_queue);
    *packet = *entry;  // shallow copy
    QUEUE_MARK_READ_DONE(&tx_queue);

    return true;
}

bool uart_scheduler(__unused repeating_timer_t *rt) {
    // LOG_INFO("Ran uart scheduler");

    ServoPacket_t packet;
    // Short-circuit to not dequeue when bus is busy
    if (packet_in_flight || !dequeue_packet(&packet))
        return true;

    uint8_t prev_interrupts = save_and_disable_interrupts();
    send_packet(packet);
    restore_interrupts(prev_interrupts);
    return true;
}
