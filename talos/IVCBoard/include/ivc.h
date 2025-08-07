#ifndef IVC_H
#define IVC_H

#include "pico/types.h"

void ivc_enqueue_packet(uint8_t data, bool is_heartbeat);

void ivc_tick();

void ivc_init();

#endif  // IVC_H
