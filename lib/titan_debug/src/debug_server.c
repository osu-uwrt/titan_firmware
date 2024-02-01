#include "titan_debug_internal.h"

#if TITAN_SAFETY

// ========================================
// Register Map
// ========================================

// Include all the required bindings
// This fragments the assorted responsibilities of the titan debug server into smaller files, making it easier to manage
// clang-format off
#include "bindings/mcu_control.h"
#include "bindings/gdb_stub.h"
#include "bindings/safety_status.h"
#include "bindings/crash_log.h"
#include "bindings/remote_cmd.h"
// clang-format on

// Map included pages defined in above header files to the global page map
reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_MCU_CONTROL_PAGE_NUM, debug_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_VERSION_STRING_PAGE_NUM),  // To be filled in at startup
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_GDB_STUB_PAGE_NUM, debug_server_gdb_stub_regs),
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_REMOTE_CMD_PAGE_NUM, debug_server_remote_cmd_regs),
    // Handle remote_cmd as ptr rather than array since we have extra length to allow terminating, so we don't want to
    // expose that to the client. Instead, lie and say it's smaaller than it actually is.
    DEFINE_PAGE_MEMMAPPED_BYTE_PTR(CANMORE_DBG_REMOTE_CMD_ARGS_PAGE_NUM, remote_cmd_args,
                                   CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN, REGISTER_PERM_READ_WRITE),
    DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(CANMORE_DBG_REMOTE_CMD_RESP_PAGE_NUM, remote_cmd_resp, REGISTER_PERM_READ_ONLY),

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
// Exported Functions
// ========================================

#include "hardware/watchdog.h"
#include "titan/safety.h"

void debug_init(reg_mapped_server_tx_func tx_func) {
    debug_server_inst.tx_func = tx_func;

    // Run any other initialization routines
    mcu_control_bindings_init();
    debug_remote_cmd_init();
    debug_register_global_remote_cmds();
}

void debug_process_message(uint8_t *msg_buffer, size_t len) {
    reg_mapped_server_handle_request(&debug_server_inst, msg_buffer, len);

    // Handle booleans defined in MCU Control Bindings
    if (enter_bootloader_on_return) {
        safety_enter_bootloader();
    }
    if (reboot_mcu_on_return) {
        watchdog_reboot(0, 0, 0);
    }
}

#endif
