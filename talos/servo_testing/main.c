#include "async_uart.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <stdio.h>

#define UART_PIN 8u
#define UART_BAUD 115200u
#define UART_TIMEOUT_MS 50

#define PARAMETER_MTU 4
#define HEADER_SIZE 2
#define HEADER_CODE 0x55
#define CHECKSUM_SIZE 1

#define MAX_PACKET_SIZE 10

#define READ_ID_PIN 0
#define SET_ID_PIN 1

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

    // Packet received correctly, make it available
    packet = rx_packet;

    LOG_INFO("Packet data: ");
    for (uint8_t i = 0; i < PARAMETER_MTU; i++)
        LOG_INFO("%x ", packet.param_buf[i]);
    LOG_INFO("\n");
}

static void on_packet_sent(enum async_uart_tx_err error) {
    if (is_read_request) {
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

    configure_pin(READ_ID_PIN);
    configure_pin(SET_ID_PIN);

    bool value = true;

    async_uart_init(pio0, 0, UART_PIN, UART_BAUD, UART_TIMEOUT_MS);

    while (true) {
        watchdog_update();
        printf("Hello World!\n");

        gpio_put(LED_PIN, value);
        value = !value;

        // uint8_t test_msg[] = { 1, 2, 3, 4, 5, 6 };
        // async_uart_write(test_msg, sizeof(test_msg) / sizeof(uint8_t), false, on_packet_sent);

        // if (gpio_get(READ_ID_PIN)) {
        //     async_uart_write()
        // }

        sleep_ms(1000);
    }
    return 0;
}
