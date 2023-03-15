#ifndef SAFETY__PROFILER_H
#define SAFETY__PROFILER_H

#include "pico.h"
#include <stdint.h>

#ifndef SAFETY_ENABLE_PROFILER
#define SAFETY_ENABLE_PROFILER 0
#endif

#if SAFETY_ENABLE_PROFILER

void profiler_push(uint32_t profiler_id);
void profiler_pop(uint32_t profiler_id);
void profiler_reset(bool check_not_popped);
void profiler_dump(void);

#else

static inline void profiler_push(__unused uint32_t profiler_id) {}
static inline void profiler_pop(__unused uint32_t profiler_id) {}
static inline void profiler_reset(__unused bool check_not_popped) {}
static inline void profiler_dump(void) {}

#endif

#endif