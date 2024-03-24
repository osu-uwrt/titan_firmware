#include "../titan_debug_internal.h"

#include "titan/safety.h"

#include <string.h>

// This file is designed to be included in the debug_server.c
// These headers each take a various page (or set of related pages) as part of the canmore debug protocol
// See canmore/debug_interface.h for more information about the underlying protocol implemented in these files

// ========================================
// Fault Data Bindings
// ========================================

static uint32_t fault_sticky = 0;
static uint32_t fault_time_upper = 0;
static uint32_t fault_time_lower = 0;
static uint32_t fault_extra_data = 0;
static uint32_t fault_line_number = 0;
static uint32_t fault_multiple_fires = 0;

// ========================================
// Safety Status Bindings
// ========================================

static uint32_t fault_idx = UINT32_MAX;

static bool safety_global_state_cb(__unused const struct reg_mapped_server_register_definition *reg,
                                   __unused bool is_write, uint32_t *data_ptr) {
    uint32_t safety_status = 0;
    if (safety_is_setup) {
        safety_status |= (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_SETUP_FIELD);
    }
    if (safety_initialized) {
        safety_status |= (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_INITIALIZED_FIELD);
    }
    if (!safety_kill_get_asserting_kill()) {
        safety_status |= (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_KILL_IS_ENABLED_FIELD);
    }
    if (*fault_list_reg != 0) {
        safety_status |= (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_FAULT_PRESENT_FIELD);
    }
    *data_ptr = safety_status;
    return true;
}

static bool fault_list_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                          uint32_t *data_ptr) {
    *data_ptr = *fault_list_reg;
    return true;
}

static bool uptime_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                      uint32_t *data_ptr) {
    static_assert(CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND == 100, "Unexpected ticks per second");
    *data_ptr = time_us_64() / 10000;
    return true;
}

static bool fault_idx_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write,
                         uint32_t *data_ptr) {
    if (!is_write) {
        *data_ptr = fault_idx;
        return true;
    }

    if (*data_ptr > MAX_FAULT_ID) {
        return false;
    }

    fault_idx = *data_ptr;

    // Lookup Fault Name
    const char *fault_name = safety_lookup_fault_id(fault_idx);
    hard_assert(debug_server_inst.num_pages > CANMORE_DBG_FAULT_NAME_PAGE_NUM);
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t *) fault_name;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.size = strlen(fault_name) + 1;

    // Store fault data
    struct fault_data *fault_entry = &safety_fault_data[fault_idx];
    fault_sticky = fault_entry->sticky_fault;
    uint64_t fault_time = to_us_since_boot(fault_entry->time);
    fault_time_lower = fault_time & (0xFFFFFFFF);
    fault_time_upper = fault_time >> 32;
    fault_extra_data = fault_entry->extra_data;
    fault_line_number = fault_entry->line;

    // Lookup Fault Filename
    const char *filename = "";
    if (fault_entry->sticky_fault) {
        // Only initialize filename if data valid (sticky fault is true)
        filename = fault_entry->filename;
    }
    hard_assert(debug_server_inst.num_pages > CANMORE_DBG_FAULT_FILENAME_PAGE_NUM);
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t *) filename;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.size = strlen(filename) + 1;

    return true;
}

static bool raise_fault_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                           uint32_t *data_ptr) {
    uint32_t fault_id = *data_ptr;
    if (fault_id > MAX_FAULT_ID) {
        return false;
    }
    safety_raise_fault(fault_id);
    return true;
}

static bool lower_fault_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                           uint32_t *data_ptr) {
    uint32_t fault_id = *data_ptr;
    if (fault_id > MAX_FAULT_ID) {
        return false;
    }
    safety_lower_fault(fault_id);
    return true;
}

// ========================================
// Exported Page
// ========================================

static const reg_mapped_server_register_def_t debug_server_safety_status_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET, safety_global_state_cb,
                             REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET, fault_list_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET, uptime_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_IDX_OFFSET, fault_idx_cb, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET, raise_fault_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET, lower_fault_cb, REGISTER_PERM_WRITE_ONLY),
};

static const reg_mapped_server_register_def_t debug_server_fault_data_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_STICKY_OFFSET, &fault_sticky, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_TIME_LOWER_OFFSET, &fault_time_lower, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_TIME_UPPER_OFFSET, &fault_time_upper, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_EXTRA_DATA_OFFSET, &fault_extra_data, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_LINE_NUMBER_OFFSET, &fault_line_number, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_MULTIPLE_FIRES_OFFSET, &fault_multiple_fires, REGISTER_PERM_READ_ONLY),
};
