#include "async_uart.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <stdio.h>

#define UART_PIN 8u
#define UART_BAUD 115200u
#define UART_TIMEOUT_MS 50

#define PARAMETER_MTU 4
#define HEADER_SIZE 2
#define HEADER_CODE 0x55
#define CHECKSUM_SIZE 1

#define READ_ID_PIN 0
#define SET_ID_PIN 1

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

typedef struct UartPacket {
    uint8_t target_id;
    uint8_t command_length;
    uint8_t command;
    uint8_t parameter_buf[PARAMETER_MTU];
    uint8_t checksum;
} UartPacket_t;

static void on_packet_sent(enum async_uart_tx_err error) {}

uint8_t calculate_checksum(UartPacket_t *packet) {
    uint data_sum = packet->target_id + packet->command_length + packet->command;
    for (uint8_t i = 0; i < packet->command_length - 3; i++) {
        data_sum += packet->parameter_buf[i];
    }

    return (uint8_t) (~data_sum);
}

void send_packet(UartPacket_t *packet) {
    uint8_t output[packet->command_length + HEADER_SIZE + CHECKSUM_SIZE];
    output[0] = output[1] = HEADER_CODE;
    output[2] = packet->target_id;
    output[3] = packet->command_length;
    output[4] = packet->command;

    for (uint8_t i = 0; i < packet->command_length - 3; i++) {
        output[i + 5] = packet->parameter_buf[i];
    }

    output[sizeof(output) / sizeof(uint8_t) - 1] = calculate_checksum(packet);

    async_uart_write(output, sizeof(output) / sizeof(uint8_t), false, on_packet_sent);
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
