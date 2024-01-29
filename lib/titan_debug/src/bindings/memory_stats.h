#include "../titan_debug_internal.h"

#include <malloc.h>

// This file is designed to be included in the debug_server.c
// These headers each take a various page (or set of related pages) as part of the canmore debug protocol
// See titan/canmore/debug_interface.h for more information about the underlying protocol implemented in these files

// ========================================
// Memory Stats Bindings
// ========================================

extern char __StackLimit;
extern char __StackTop;
extern char __StackBottom;
extern char __data_start__;
extern char end;

static uint32_t mem_stats_total_mem = 0;
static uint32_t mem_stats_heap_use = 0;
static uint32_t mem_stats_stack_use = 0;
static uint32_t mem_stats_static_use = 0;
static uint32_t mem_stats_arena = 0;
static uint32_t mem_stats_ordblks = 0;
static uint32_t mem_stats_hblks = 0;
static uint32_t mem_stats_hblkhd = 0;
static uint32_t mem_stats_uordblks = 0;
static uint32_t mem_stats_fordblks = 0;
static uint32_t mem_stats_keepcost = 0;

static bool mem_stats_capture_cb(__unused const struct reg_mapped_server_register_definition *reg,
                                 __unused bool is_write, __unused uint32_t *data_ptr) {
    struct mallinfo mi = mallinfo();

    mem_stats_total_mem = (&__StackTop - &__data_start__);
    mem_stats_heap_use = (&__StackLimit - &end);
    mem_stats_stack_use = (&__StackTop - &__StackBottom);
    mem_stats_static_use = (&end - &__data_start__);

    mem_stats_arena = mi.arena;
    mem_stats_ordblks = mi.ordblks;
    mem_stats_hblks = mi.hblks;
    mem_stats_hblkhd = mi.hblkhd;
    mem_stats_uordblks = mi.uordblks;
    mem_stats_fordblks = mi.fordblks;
    mem_stats_keepcost = mi.keepcost;

    return true;
}

// ========================================
// Exported Page
// ========================================

static const reg_mapped_server_register_def_t debug_server_memory_stats_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MEM_STATS_CAPTURE_OFFSET, mem_stats_capture_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_TOTAL_MEM_OFFSET, &mem_stats_total_mem, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_HEAP_USE_OFFSET, &mem_stats_heap_use, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_STACK_USE_OFFSET, &mem_stats_stack_use, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_STATIC_USE_OFFSET, &mem_stats_static_use, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_ARENA_OFFSET, &mem_stats_arena, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_ORDBLKS_OFFSET, &mem_stats_ordblks, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_HBLKS_OFFSET, &mem_stats_hblks, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_HBLKHD_OFFSET, &mem_stats_hblkhd, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_UORDBLKS_OFFSET, &mem_stats_uordblks, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_FORDBLKS_OFFSET, &mem_stats_fordblks, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MEM_STATS_KEEPCOST_OFFSET, &mem_stats_keepcost, REGISTER_PERM_READ_ONLY),
};
