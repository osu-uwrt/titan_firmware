#include "ivc.h"
#include "tx.h"

// [ 7 ][ 6 ][ 5 ][ 4 ][ 3 ][ 2 ][ 1 ][ 0 ] => bus

// Calculate CRC-4
uint8_t calculate_crc(uint8_t packet) {
    uint8_t crc = 0;
    for (int i = DATA_SIZE - 1; i >= 0; i--) {
        crc <<= 1;
        crc |= (packet >> i) & 0x01;

        if (crc & 0x10)
            crc ^= CRC_POLY;
    }

    return crc & 0x0F;  // Only care about lower 4 bits; note: this will need changed if CRC_SIZE is different
}

// message good
void send_ack(ivc_context_t *ctx) {
    // respond with 0
    tx_enqueue_data(0x00);
}

// need retransmission
void send_nack(ivc_context_t *ctx) {
    tx_enqueue_data(0x01);
}

void process_packet(ivc_context_t *ctx) {
    uint8_t received_crc = (ctx->rx.packet_to_process >> DATA_SIZE) & 0x0F;
    uint8_t calculated_crc = calculate_crc((uint8_t) ctx->rx.packet_to_process & 0xFF);
    if (received_crc == calculated_crc) {
        send_ack(ctx);
    }
    else {
        send_nack(ctx);
    }
}
