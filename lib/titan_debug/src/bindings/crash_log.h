#include "../titan_debug_internal.h"

#include "titan/safety.h"
#include "titan/safety_magic_values.h"

// This file is designed to be included in the debug_server.c
// These headers each take a various page (or set of related pages) as part of the canmore debug protocol
// See canmore/debug_interface.h for more information about the underlying protocol implemented in these files

// ========================================
// Crash Log Bindings
// ========================================

static uint32_t selected_prev_index = UINT32_MAX;
static uint32_t selected_prev_reset_reason = 0;
static uint32_t selected_prev_fault_list = 0;
static uint32_t selected_prev_uptime = 0;
static uint32_t selected_prev_scratch_1 = 0;
static uint32_t selected_prev_scratch_2 = 0;

static bool crash_count_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                           uint32_t *data_ptr) {
    uint32_t crash_count = 0;
    crash_count |= (crash_data.crash_counter.total_crashes & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_MASK)
                   << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_SHIFT;
    crash_count |= (crash_data.crash_counter.panic_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_MASK)
                   << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_SHIFT;
    crash_count |= (crash_data.crash_counter.hard_fault_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_MASK)
                   << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_SHIFT;
    crash_count |= (crash_data.crash_counter.assert_fail_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_MASK)
                   << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_SHIFT;
    *data_ptr = crash_count;
    return true;
}

static bool has_wrapped_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                           uint32_t *data_ptr) {
    *data_ptr = crash_data.header.flags.log_wrapped;
    return true;
}

static bool prev_count_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                          uint32_t *data_ptr) {
    if (crash_data.header.flags.log_wrapped) {
        *data_ptr = SAFETY_NUM_CRASH_LOG_ENTRIES;
    }
    else {
        *data_ptr = crash_data.header.next_entry;
    }
    return true;
}

static bool prev_index_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write,
                          uint32_t *data_ptr) {
    if (!is_write) {
        *data_ptr = selected_prev_index;
        return true;
    }

    uint32_t idx = *data_ptr;
    uint32_t next_entry = crash_data.header.next_entry;
    if (crash_data.header.flags.log_wrapped) {
        if (idx >= SAFETY_NUM_CRASH_LOG_ENTRIES) {
            // Attempting to access index larger than crash log
            return false;
        }
        // So idx is right now the number of entries back we need to go
        if (idx < next_entry) {
            // So if we're in position 3, and idx 0 is requested, return position 2
            idx = next_entry - 1 - idx;
        }
        else {
            // And if we're in position 3, and idx 3 is requested, we need to return num entries - 1
            // First allow idx to be the number of positions from end of crash log
            idx = idx - next_entry;
            idx = SAFETY_NUM_CRASH_LOG_ENTRIES - 1 - idx;
        }
    }
    else {
        if (idx >= next_entry) {
            // That crash data entry isn't valid yet, return error
            return false;
        }
        // Set the index in reverse order (most recent reset is index 0)
        idx = next_entry - 1 - idx;
    }

    // We've computed the corrected index, set the indexes accordingly
    struct crash_log_entry *entry = &crash_data.crash_log[idx];
    selected_prev_index = *data_ptr;
    switch (entry->reset_reason) {
    case CLEAN_BOOT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_CLEAN_BOOT_VALUE;
        break;
    case UNKNOWN_SAFETY_PREINIT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_PREINIT_VALUE;
        break;
    case UNKNOWN_SAFETY_ACTIVE:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_ACTIVE_VALUE;
        break;
    case PANIC:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_PANIC_VALUE;
        break;
    case HARD_FAULT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_FAULT_VALUE;
        break;
    case ASSERT_FAIL:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_ASSERT_FAIL_VALUE;
        break;
    case HARD_ASSERT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_ASSERT_VALUE;
        break;
    case WATCHDOG_TIMEOUT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_WATCHDOG_TIMEOUT_VALUE;
        break;
    case CORE1_TIMEOUT:
        selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_CORE1_TIMEOUT_VALUE;
        break;
    default:
        selected_prev_reset_reason = entry->reset_reason;  // If it's not in this list, fallback to the raw value
    }

    selected_prev_fault_list = entry->faults;
    selected_prev_uptime = entry->uptime;

    // If clean boot, do another translation
    if (entry->reset_reason == CLEAN_BOOT) {
        switch (entry->scratch_1) {
        case CLEAN_RESET_TYPE_POR:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_POR_VALUE;
            break;
        case CLEAN_RESET_TYPE_RUN:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_RUN_VALUE;
            break;
        case CLEAN_RESET_TYPE_PSM:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_PSM_VALUE;
            break;
        case CLEAN_RESET_TYPE_SOFTWARE:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_SOFTWARE_VALUE;
            break;
        case CLEAN_RESET_TYPE_UNK_WDG:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_UNK_WDG_VALUE;
            break;
        case CLEAN_RESET_TYPE_BOOTLOADER:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_BOOTLOADER_VALUE;
            break;
        case CLEAN_RESET_TYPE_BOOTROM:
            selected_prev_scratch_1 = CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_BOOTROM_VALUE;
            break;
        default:
            selected_prev_scratch_1 = entry->scratch_1;  // If it's not in this list, fallback to the raw value
        }
    }
    else {
        selected_prev_scratch_1 = entry->scratch_1;
    }
    selected_prev_scratch_2 = entry->scratch_2;
    return true;
}

// ========================================
// Exported Page
// ========================================

static const reg_mapped_server_register_def_t debug_server_crash_log_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_CRASH_COUNT_OFFSET, crash_count_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_HAS_WRAPPED_OFFSET, has_wrapped_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_PREV_COUNT_OFFSET, prev_count_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, prev_index_cb, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET, &selected_prev_reset_reason,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET, &selected_prev_fault_list,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET, &selected_prev_uptime, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET, &selected_prev_scratch_1,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET, &selected_prev_scratch_2,
                          REGISTER_PERM_READ_ONLY),
};
