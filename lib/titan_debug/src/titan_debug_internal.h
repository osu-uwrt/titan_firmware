#ifndef TITAN_DEBUG_INTERNAL_H_
#define TITAN_DEBUG_INTERNAL_H_

#include "pico/platform.h"
#include "titan/canmore.h"
#include "titan/debug.h"

// Defined in debug_server.c
// Used by various bindings to modify the underlying server and page mappings
extern reg_mapped_server_inst_t debug_server_inst;
extern reg_mapped_server_page_def_t debug_server_pages[];

// Remote Command Function Handler
void debug_remote_cmd_init(void);
int debug_remote_cmd_handle(const char *args, size_t resp_size, char *resp);

// Handles registration for global remote commands (rather than passing complex data objects over titan debug)
void debug_register_global_remote_cmds(void);

#endif
