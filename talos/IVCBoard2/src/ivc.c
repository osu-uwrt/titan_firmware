#include "ivc.h"

#include "consensus.h"
#include "fft/fft.h"
#include "rx.h"
#include "tx.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <stdint.h>

// #define SYSCLK_HZ clock_get_hz(clk_sys)
// #define PWM_WRAP_VALUE ((int) (((float) SYSCLK_HZ) / 5000 / 255.0f) + 1.0f)

bool is_talos = false;
ivc_context_t context = { 0 };

void swap_buffer_handler() {
    // recv_data.fft_target = recv_data.buffer_select;
    // recv_data.buffer_select = !recv_data.buffer_select;
    // fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
    context.rx.recv.fft_target = context.rx.recv.buffer_select;
    context.rx.recv.buffer_select = !context.rx.recv.buffer_select;
    fft_sample(context.rx.recv.swap_buffers[context.rx.recv.buffer_select]);
    // sample_t sample = rx_observe();
    // LOG_INFO("pushing %hhu", sample);
    // consensus_push(sample);
}

void ivc_init() {
    tx_init(&context);
    rx_init(&context, swap_buffer_handler);

    gpio_init(BOARD_ID_PIN);
    gpio_set_dir(BOARD_ID_PIN, GPIO_IN);

    is_talos = !gpio_get(BOARD_ID_PIN);
    LOG_INFO("This board %s talos", is_talos ? "is" : "is not");
}

// Calculate CRC-4
uint8_t calculate_crc(uint8_t packet) {
    // uint16_t packet_agumented = ((uint16_t) packet) << CRC_SIZE;
    uint8_t crc = 0;
    for (int i = DATA_SIZE - 1; i >= 0; i--) {
        crc <<= 1;
        crc |= (packet >> i) & 0x01;

        if (crc & 0x10)
            crc ^= CRC_POLY;
    }

    return crc & 0x0F;  // Only care about lower 4 bits; note: this will need changed if CRC_SIZE is different
}

void log_sample(sample_t s) {
    switch (s) {
    case NONE:
        LOG_INFO("SAMPLE = NONE");
        break;
    case HIGH:
        LOG_INFO("SAMPLE = HIGH");
        break;
    case LOW:
        LOG_INFO("SAMPLE = LOW");
        break;
    case SYNC:
        LOG_INFO("SAMPLE = SYNC");
        break;
    }
}

// void tick() {
//     sample_t sample = rx_sample();
//     // log_sample(sample);
//     //  sync denotes a packet is starting
//     if (sample == SYNC) {
//         LOG_INFO("got sync");
//         //   send ack probably
//         //  handle_incoming_packet();
//         attempt_packet_read();
//     }
//     else {
//         // tx_encode_data();
//         attempt_writing();
//     }
// }

void ivc_tick() {
    consensus_update(&context.consensus, rx_observe(&context));
    attempt_reading(&context);
    attempt_writing(&context);
    // hello
}

// send wake tone
// start listening
// sample and add to consensus buf
// once consenus, repeat at symbol period
