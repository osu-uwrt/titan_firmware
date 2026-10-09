#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "ivc.h"

void send_nack(ivc_context_t *ctx);
void send_ack(ivc_context_t *ctx);
void process_packet(ivc_context_t *ctx);
uint8_t calculate_crc(uint8_t packet);

#endif  // PROTOCOL_H
