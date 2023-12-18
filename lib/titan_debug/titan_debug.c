#include "pico/sync.h"
#include "pico/unique_id.h"
#include "titan/debug.h"

#include <malloc.h>
#include <string.h>

#if TITAN_SAFETY
#include "hardware/watchdog.h"
#include "titan/safety.h"
#include "titan/safety_magic_values.h"
#include "titan/version.h"

#ifdef MICRO_ROS_TRANSPORT_CAN
#include "driver/canbus.h"
#endif

extern reg_mapped_server_inst_t debug_server_inst;  // Allows us to refer to the debug server later

// ========================================
// MCU Control Bindings
// ========================================

static uint32_t mcu_control_magic_value = CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE;
static uint32_t mcu_control_major_version;
static uint32_t mcu_control_minor_version;
static uint32_t mcu_control_release_type;
static bool enter_bootloader_on_return = false;
static bool reboot_mcu_on_return = false;
static uint32_t gdb_stub_memory_data = 0;

static union {
    pico_unique_board_id_t id_pico;
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_word) == PICO_UNIQUE_BOARD_ID_SIZE_BYTES,
              "Flash unique ID does not match expected size");

static bool enter_bl_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                        __unused uint32_t *data_ptr) {
    enter_bootloader_on_return = true;
    return true;
}

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                          __unused uint32_t *data_ptr) {
    reboot_mcu_on_return = true;
    return true;
}

static bool gdb_stub_write_mem_cb(__unused const struct reg_mapped_server_register_definition *reg,
                                  __unused bool is_write, uint32_t *data_ptr) {
    uintptr_t addr = *data_ptr;
    if (addr % 4 != 0) {
        // Don't allow unaligned writes
        return false;
    }

    if (addr >= SRAM_BASE && addr < SRAM_END)  // Striped addresses (& scratch_x + scratch_y)
        ;
    else if (addr >= SRAM0_BASE && addr < 0x21040000)  // Non-striped RAM
        ;
    else if (addr >= 0x40000000 && addr < 0x40070000)  // APB Peripherals
        ;
    else if (addr >= 0x50000000 && addr < 0x50400000)  // AHB Peripherals
        ;
    else if (addr >= SIO_BASE && addr < SIO_BASE + 0x180)  // SIO Peripherals
        ;
    else if (addr >= PPB_BASE + 0x0000e000 && addr < PPB_BASE + 0x0000f000)  // ARM Cortex SCS
        ;
    else
        return false;

    // Perform the write
    uint32_t *mem_ptr = (uint32_t *) addr;
    *mem_ptr = gdb_stub_memory_data;
    return true;
}

static bool gdb_stub_read_mem_cb(const struct reg_mapped_server_register_definition *reg, bool is_write,
                                 uint32_t *data_ptr) {
    register uint32_t addr = 0;
    bool is_pc_read = false;
    bool is_sp_read = false;
    bool is_lr_read = false;

    // This function is extra convoluted since we want to capture the memory and PC/SP at the same location, so we get
    // accurate stack reconstruction. Yay ARM ABI!

    // Do register decoding
    if (debug_server_inst.num_pages < CANMORE_DBG_GDB_STUB_PAGE_NUM ||
        debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].page_type != PAGE_TYPE_REGISTER_MAPPED)
        return false;
    size_t reg_cnt = debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].type.reg_mapped.num_registers;
    const reg_mapped_server_register_def_t *page_def =
        debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].type.reg_mapped.reg_array;

    // This is a read word address
    if (reg_cnt > CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET &&
        reg == &page_def[CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET]) {
        // Read word will write the value to fetch, if it's a read, they're using it wrong
        if (!is_write) {
            return false;
        }
        addr = *data_ptr;

        if (addr % 4 != 0) {
            // Address must be word aligned
            return false;
        }

        // Check that address is in range
        static_assert(ROM_BASE == 0, "Assuming rom tsarts at 0 to silence compiler warning");

        if (addr >= SRAM_BASE && addr < SRAM_END)  // Striped addresses (& scratch_x + scratch_y)
            ;
        else if (addr >= XIP_MAIN_BASE && addr < XIP_MAIN_BASE + PICO_FLASH_SIZE_BYTES)  // Flash normal
            ;
        else if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + PICO_FLASH_SIZE_BYTES)  // Flash noalloc
            ;
        else if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + PICO_FLASH_SIZE_BYTES)  // Flash noalloc
            ;
        else if (addr >= XIP_NOCACHE_NOALLOC_BASE && addr < XIP_NOCACHE_NOALLOC_BASE + PICO_FLASH_SIZE_BYTES)
            ;
        else if (addr < (16 * 1024))  // ROM (Hardcoded from datasheet saying that ROM is 16K)
            ;
        else if (addr >= SRAM0_BASE && addr < 0x21040000)  // Non-striped RAM
            ;
        else if (addr >= 0x40000000 && addr < 0x40070000)  // APB Peripherals
            ;
        else if (addr >= 0x50000000 && addr < 0x50400000)  // AHB Peripherals
            ;
        else if (addr >= SIO_BASE && addr < SIO_BASE + 0x180)  // SIO Peripherals
            ;
        else if (addr >= PPB_BASE + 0x0000e000 && addr < PPB_BASE + 0x0000f000)  // ARM Cortex SCS
            ;
        else {
            // Address out of range
            return false;
        }

        // If address is in flash, only do noalloc reads so we don't muddy up cache
        if ((addr & 0xFC000000) == XIP_BASE) {
            addr = XIP_NOALLOC_BASE + (addr & 0x00FFFFFF);
        }
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET]) {
        is_pc_read = true;
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET]) {
        is_sp_read = true;
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET]) {
        is_lr_read = true;
    }
    else {
        // We don't know what it is
        return false;
    }

    // Perform the read. This must be protected so the compiler doesn't optimize it away
    __dsb();
    __isb();
    register uint32_t val;
    if (is_pc_read)
        pico_default_asm_volatile("mov %0, pc\n" : "=r"(val));
    else if (is_sp_read)
        pico_default_asm_volatile("mov %0, sp\n" : "=r"(val));
    else if (is_lr_read)
        pico_default_asm_volatile("mov %0, lr\n" : "=r"(val));
    else
        pico_default_asm_volatile("ldr %0, [%1]" : "=r"(val) : "r"(addr));
    __dsb();
    __isb();

    if (is_sp_read || is_pc_read || is_lr_read) {
        if (is_write) {
            return false;  // PC and SP reads must be reads
        }
        *data_ptr = val;
    }
    else {
        // It's a memory fetch, store into the appropriate register
        gdb_stub_memory_data = val;
    }
    return true;
}

#ifdef MICRO_ROS_TRANSPORT_CAN
static bool can_intr_en_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                           __unused uint32_t *data_ptr) {
    canbus_reenable_intr();
    return true;
}

static bool can_fifo_clear_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                              __unused uint32_t *data_ptr) {
    canbus_fifo_clear();
    return true;
}

static bool can_reset_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                         __unused uint32_t *data_ptr) {
    canbus_reset();
    return true;
}
#endif

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

static bool fault_idx_cb(const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr);
// Must be defined after debug_server_pages

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

static const reg_mapped_server_register_def_t debug_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET, &mcu_control_magic_value, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, enter_bl_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID, &mcu_control_flash_id.id_word[0],
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID, &mcu_control_flash_id.id_word[1],
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAJOR_VERSION_OFFSET, &mcu_control_major_version,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MINOR_VERSION_OFFSET, &mcu_control_minor_version,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_RELEASE_TYPE_OFFSET, &mcu_control_release_type,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
#ifdef MICRO_ROS_TRANSPORT_CAN
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_CAN_INTR_EN_OFFSET, can_intr_en_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_CAN_FIFO_CLEAR_OFFSET, can_fifo_clear_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_CAN_RESET_OFFSET, can_reset_cb, REGISTER_PERM_WRITE_ONLY),
#endif
};

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

static const reg_mapped_server_register_def_t debug_server_gdb_stub_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET, gdb_stub_read_mem_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_WRITE_WORD_ADDR_OFFSET, gdb_stub_write_mem_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_GDB_STUB_MEMORY_DATA_OFFSET, &gdb_stub_memory_data, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
};

static const reg_mapped_server_register_def_t debug_server_safety_status_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET, safety_global_state_cb,
                             REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET, fault_list_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET, uptime_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_FAULT_IDX_OFFSET, fault_idx_cb, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET, raise_fault_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET, lower_fault_cb, REGISTER_PERM_WRITE_ONLY),
};

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

static const reg_mapped_server_register_def_t debug_server_fault_data_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_STICKY_OFFSET, &fault_sticky, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_TIME_LOWER_OFFSET, &fault_time_lower, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_TIME_UPPER_OFFSET, &fault_time_upper, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_EXTRA_DATA_OFFSET, &fault_extra_data, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_LINE_NUMBER_OFFSET, &fault_line_number, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_FAULT_DATA_MULTIPLE_FIRES_OFFSET, &fault_multiple_fires, REGISTER_PERM_READ_ONLY),
};

static reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_MCU_CONTROL_PAGE_NUM, debug_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_VERSION_STRING_PAGE_NUM),  // To be filled in at startup
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_MEM_STATS_PAGE_NUM, debug_server_memory_stats_regs),
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_GDB_STUB_PAGE_NUM, debug_server_gdb_stub_regs),

    // Safety Status Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_SAFETY_STATUS_PAGE_NUM, debug_server_safety_status_regs),
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_CRASH_LOG_PAGE_NUM, debug_server_crash_log_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_FAULT_NAME_PAGE_NUM),  // Filled in by fault idx register cb
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_FAULT_DATA_PAGE_NUM, debug_server_fault_data_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_FAULT_FILENAME_PAGE_NUM),  // Filled in by fault idx register cb
};

reg_mapped_server_inst_t debug_server_inst = {
    .page_array = debug_server_pages,
    .num_pages = sizeof(debug_server_pages) / sizeof(*debug_server_pages),
    .control_interface_mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL,
};

// ========================================
// Must be after debug_server_pages
// ========================================

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
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t *) filename;
    debug_server_pages[CANMORE_DBG_FAULT_FILENAME_PAGE_NUM].type.mem_mapped_byte.size = strlen(filename) + 1;

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
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t *) FULL_BUILD_TAG;
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
