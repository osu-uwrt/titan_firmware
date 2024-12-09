#include "../titan_debug_internal.h"

// ========================================
// Remote Command Bindings
// ========================================

// Add extra length to the args so we can pad in 0s to enforce that the command decoding stops at the end of the string
static uint8_t remote_cmd_args[CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN + 2];
static uint8_t remote_cmd_resp[CANMORE_DBG_REMOTE_CMD_RESP_MAX_LEN];

static bool remote_cmd_execute(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                               uint32_t *data_ptr) {
    // Pad 0s to ensure that the remote command handler won't decode past the end of the array
    // This ensures that the last argument is null terminated, and then an empty string is read after that null
    // termination.
    remote_cmd_args[CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN] = 0;
    remote_cmd_args[CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN + 1] = 0;

    // Execute the command then store the return code
    int rc = debug_remote_cmd_handle((char *) remote_cmd_args, sizeof(remote_cmd_resp), (char *) remote_cmd_resp);
    *data_ptr = (uint32_t) rc;

    // Clear the previous command to prevent it from accidentally running again if this is read multiple times
    remote_cmd_args[0] = 0;

    return true;
}

// ========================================
// Exported Page
// ========================================

static const reg_mapped_server_register_def_t debug_server_remote_cmd_regs[] = { DEFINE_REG_EXEC_CALLBACK(
    CANMORE_DBG_REMOTE_CMD_EXECUTE_OFFSET, remote_cmd_execute, REGISTER_PERM_READ_ONLY) };
