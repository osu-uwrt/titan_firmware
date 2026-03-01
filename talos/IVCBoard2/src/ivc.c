#include "ivc.h"

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

// each robot sends a ping to initiate comms
// each robot has a different ping frequency
// talos has precedence

// calc crc before send and upon receive

// before transmission, listen for comm init tone.
// if comm init tone, dont transmit message and listen.
// else transmit

// maintain ring buffer of samples and do consensus sampling at 7 times the symbol frequency
// uint8_t sample_buffers[2][NSAMP] = { 0 };
bool is_talos = false;
comm_state_t state = IDLE;

symbol_clock_t sym_clk = { 0 };

void ivc_init() {
    tx_init();
    rx_init();

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

// void tick() {
//     switch (state) {
//     case IDLE:
//         break;  // nothing to do
//     case SYNC_FOUND:
//         // send ack
//         break;
//     case LINK_ESTABLISHED:
//         // set stuff
//         break;
//     case PACKET_IN_FLIGHT:
//         // read samples
//         break;
//     case TRANSMITTING:
//         break;
//     case RECEIVING:
//         break;
//     }
// }

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

void tick() {
    sample_t sample = rx_sample();
    // log_sample(sample);
    //  sync denotes a packet is starting
    if (sample == SYNC) {
        LOG_INFO("got sync");
        //   send ack probably
        //  handle_incoming_packet();
        attempt_packet_read();
    }
    else {
        // tx_encode_data();
        attempt_writing();
    }
}

void new_tick() {
    consensus_push(rx_observe());

    sample_t sample;
    if (consensus_stable(&sample)) {
        if (sample == SYNC) {
            consensus_reset();
            //   LOG_INFO("RECEIVED SYNC PULSE");
            listen_for_packet();
        }
    }
    attempt_writing();
}

// send wake tone
// start listening
// sample and add to consensus buf
// once consenus, repeat at symbol period
