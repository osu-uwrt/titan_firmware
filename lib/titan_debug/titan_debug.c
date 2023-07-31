#include <string.h>
#include "pico/unique_id.h"

#include "titan/debug.h"

#if TITAN_SAFETY
#include "titan/safety.h"
#include "titan/safety_magic_values.h"
#include "titan/version.h"
#include "hardware/watchdog.h"

// ========================================
// MCU Control Bindings
// ========================================

static uint32_t mcu_control_magic_value = CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE;
static uint32_t mcu_control_major_version;
static uint32_t mcu_control_minor_version;
static uint32_t mcu_control_release_type;
static bool enter_bootloader_on_return = false;
static bool reboot_mcu_on_return = false;

static union {
    pico_unique_board_id_t id_pico;
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_word) == PICO_UNIQUE_BOARD_ID_SIZE_BYTES, "Flash unique ID does not match expected size");

static bool enter_bl_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    enter_bootloader_on_return = true;
    return true;
}

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    reboot_mcu_on_return = true;
    return true;
}

// ========================================
// Safety Status Bindings
// ========================================

static uint32_t fault_name_idx = UINT32_MAX;

static bool safety_global_state_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
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

static bool fault_list_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    *data_ptr = *fault_list_reg;
    return true;
}

static bool uptime_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    static_assert(CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND == 100, "Unexpected ticks per second");
    *data_ptr = time_us_64() / 10000;
    return true;
}

static bool fault_name_idx_cb(const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr);
// Must be defined after debug_server_pages

static bool raise_fault_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    uint32_t fault_id = *data_ptr;
    if (fault_id > MAX_FAULT_ID) {
        return false;
    }
    safety_raise_fault(fault_id);
    return true;
}

static bool lower_fault_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    uint32_t fault_id = *data_ptr;
    if (fault_id > MAX_FAULT_ID) {
        return false;
    }
    safety_lower_fault(fault_id);
    return true;
}

// ========================================
// Crash Log Bindings
// ========================================

static uint32_t selected_prev_index = UINT32_MAX;
static uint32_t selected_prev_reset_reason = 0;
static uint32_t selected_prev_fault_list = 0;
static uint32_t selected_prev_uptime = 0;
static uint32_t selected_prev_scratch_1 = 0;
static uint32_t selected_prev_scratch_2 = 0;

static bool crash_count_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    uint32_t crash_count = 0;
    crash_count |= (crash_data.crash_counter.total_crashes & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_MASK) << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_SHIFT;
    crash_count |= (crash_data.crash_counter.panic_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_MASK) << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_SHIFT;
    crash_count |= (crash_data.crash_counter.hard_fault_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_MASK) << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_SHIFT;
    crash_count |= (crash_data.crash_counter.assert_fail_count & CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_MASK) << CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_SHIFT;
    *data_ptr = crash_count;
    return true;
}

static bool has_wrapped_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    *data_ptr = crash_data.header.flags.log_wrapped;
    return true;
}

static bool prev_count_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, uint32_t *data_ptr) {
    if (crash_data.header.flags.log_wrapped) {
        *data_ptr = SAFETY_NUM_CRASH_LOG_ENTRIES;
    }
    else {
        *data_ptr = crash_data.header.next_entry;
    }
    return true;
}

static bool prev_index_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr) {
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
        case IN_ROS_TRANSPORT_LOOP:
            selected_prev_reset_reason = CANMORE_DBG_CRASH_LOG_RESET_REASON_TIMEOUT_DURING_ROS_VALUE;
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
            default:
                selected_prev_scratch_1 = entry->reset_reason;  // If it's not in this list, fallback to the raw value
        }
    }
    else {
        selected_prev_scratch_1 = entry->scratch_1;
    }
    selected_prev_scratch_2 = entry->scratch_2;
    return true;
}

// ========================================
// Register Map
// ========================================

static reg_mapped_server_register_def_t debug_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET, &mcu_control_magic_value, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, enter_bl_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID, &mcu_control_flash_id.id_word[0], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID, &mcu_control_flash_id.id_word[1], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAJOR_VERSION_OFFSET, &mcu_control_major_version, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MINOR_VERSION_OFFSET, &mcu_control_minor_version, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_RELEASE_TYPE_OFFSET, &mcu_control_release_type, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
};

static reg_mapped_server_register_def_t debug_server_safety_status_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET, safety_global_state_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET, fault_list_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET, uptime_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_NAME_IDX_OFFSET, fault_name_idx_cb, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET, raise_fault_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET, lower_fault_cb, REGISTER_PERM_WRITE_ONLY),
};

static reg_mapped_server_register_def_t debug_server_crash_log_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_CRASH_COUNT_OFFSET, crash_count_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_HAS_WRAPPED_OFFSET, has_wrapped_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_PREV_COUNT_OFFSET, prev_count_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, prev_index_cb, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET, &selected_prev_reset_reason, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET, &selected_prev_fault_list, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET, &selected_prev_uptime, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET, &selected_prev_scratch_1, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET, &selected_prev_scratch_2, REGISTER_PERM_READ_ONLY),
};

static reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_MCU_CONTROL_PAGE_NUM, debug_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_VERSION_STRING_PAGE_NUM),  // To be filled in at startup

    // Safety Status Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_SAFETY_STATUS_PAGE_NUM, debug_server_safety_status_regs),
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_CRASH_LOG_PAGE_NUM, debug_server_crash_log_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_FAULT_NAME_PAGE_NUM),  // Filled in by fault name idx register
};

static reg_mapped_server_inst_t debug_server_inst = {
    .page_array = debug_server_pages,
    .num_pages = sizeof(debug_server_pages)/sizeof(*debug_server_pages),
    .control_interface_mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL,
};

// ========================================
// Must be after debug_server_pages
// ========================================

static bool fault_name_idx_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr) {
    if (!is_write) {
        *data_ptr = fault_name_idx;
        return true;
    }

    if (*data_ptr > MAX_FAULT_ID) {
        return false;
    }

    fault_name_idx = *data_ptr;
    const char* fault_name = safety_lookup_fault_id(fault_name_idx);
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t*) fault_name;
    debug_server_pages[CANMORE_DBG_FAULT_NAME_PAGE_NUM].type.mem_mapped_byte.size = strlen(fault_name) + 1;
    return true;
}

// ========================================
// Exported Functions
// ========================================

void debug_init(reg_mapped_server_tx_func tx_func) {
    debug_server_inst.tx_func = tx_func;

    // Fill out version
    mcu_control_major_version = MAJOR_VERSION;
    mcu_control_minor_version = MINOR_VERSION;
    mcu_control_release_type = RELEASE_TYPE;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t*) FULL_BUILD_TAG;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.size = strlen(FULL_BUILD_TAG) + 1;

    // Fill out board id
    pico_get_unique_board_id(&mcu_control_flash_id.id_pico);
}

void debug_process_message(uint8_t *msg_buffer, size_t len) {
    reg_mapped_server_handle_request(&debug_server_inst, msg_buffer, len);
    if (enter_bootloader_on_return) {
        safety_enter_bootloader();
    }
    if (reboot_mcu_on_return) {
        watchdog_reboot(0, 0, 0);
    }
}

#endif
