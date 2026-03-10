#include "ivc.h"
#include "tx.h"

// [ 7 ][ 6 ][ 5 ][ 4 ][ 3 ][ 2 ][ 1 ][ 0 ] => bus

// message good
void send_ack(ivc_context_t *ctx) {}

// need retransmission
void send_nack(ivc_context_t *ctx) {}
