#ifndef IVC_H
#define IVC_H

#include "pico/types.h"

void ivc_tx(uint8_t data);

void ivc_tick();

void ivc_init();

#endif  // IVC_H
