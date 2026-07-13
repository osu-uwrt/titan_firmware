#ifndef CANMORE_H
#define CANMORE_H

#include "ivc.h"
#include "tx.h"

#include <stdint.h>
#include <stdio.h>

void canmore_serve();
void register_canmore_commands(ivc_context_t *ctx);

#endif  // CANMORE_H
