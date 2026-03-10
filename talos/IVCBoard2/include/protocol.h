#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "ivc.h"

void send_nack(ivc_context_t *ctx);
void send_ack(ivc_context_t *ctx);
void process_packet(ivc_context_t *ctx);

#endif // PROTOCOL_H