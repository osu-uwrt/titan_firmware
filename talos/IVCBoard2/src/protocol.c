#include "ivc.h"
#include "tx.h"

// [ 7 ][ 6 ][ 5 ][ 4 ][ 3 ][ 2 ][ 1 ][ 0 ] => bus

// Calculate CRC-4
static uint8_t calculate_crc(uint8_t packet) {
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

void process_packet(ivc_context_t *ctx) {}

// message good
void send_ack(ivc_context_t *ctx) {}

// need retransmission
void send_nack(ivc_context_t *ctx) {}
