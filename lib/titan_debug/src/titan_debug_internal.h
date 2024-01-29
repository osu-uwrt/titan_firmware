#ifndef TITAN_DEBUG_INTERNAL_H_
#define TITAN_DEBUG_INTERNAL_H_

#include "pico/platform.h"
#include "titan/canmore.h"
#include "titan/debug.h"

// Defined in debug_server.c
// Used by various bindings to modify the underlying server and page mappings
extern reg_mapped_server_inst_t debug_server_inst;
extern reg_mapped_server_page_def_t debug_server_pages[];

#endif
