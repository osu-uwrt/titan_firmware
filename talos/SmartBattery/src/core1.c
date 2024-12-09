#define RUNNING_ON_CORE1
#include "core1.h"

#include "bq40z80.h"
#include "pio_smbus.h"
#include "safety_interface.h"

#include "pico/multicore.h"
#include "titan/debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BQ40Z80_REFRESH_TIME_MS 200
#define FIFO_REQ_VALUE_WIDTH 24

static uint8_t sbh_mcu_serial;

typedef union sio_fifo_req {
    struct __packed {
        enum __packed cmd_type {
            FIFO_CMD_POWER_CYCLE,
            FIFO_CMD_KILL_POWER,
            FIFO_CMD_READ_CYCLE_COUNT,
            FIFO_CMD_READ_STATE_OF_HEALTH,
            FIFO_CMD_READ_CAPACITY,
            FIFO_CMD_READ_SIDE_DETECT,
            FIFO_CMD_READ_GAUGING_STATUS,
            FIFO_CMD_READ_CHARGING_STATUS,
            FIFO_CMD_DBG_COMMAND
        } type;
        uint32_t arg : FIFO_REQ_VALUE_WIDTH;
    };
    uint32_t raw;
} sio_fifo_req_t;

static_assert(sizeof(sio_fifo_req_t) == sizeof(uint32_t), "FIFO Command did not pack properly");

// ================================================
// Shared Memory Definitions
// ================================================
#define READ_STATE_IDLE 0
#define READ_STATE_IN_PROGRESS 1
#define READ_STATE_UNREAD 2

/**
 * @brief Shared memory for sending battery status from core 1 to core 0
 *
 * @attention Only be modified or read when lock is held
 *
 */
static volatile struct core1_status_shared_mem {
    /**
     * @brief spin lock to protect battery status transferring across cores
     *
     */
    spin_lock_t *lock;

    batt_state_t state;
    struct bq_battery_info_t batt_info;
    struct bq_mfg_info_t mfg_info;

    struct req_results {
        uint8_t side_detect_read_state;
        bool side_detect;
        bool reg_read_pending;
        bool reg_read_okay;
        uint32_t reg_read_result;
    } req_results;

} shared_status = { .state = BATT_STATE_UNINITIALIZED };

// Objects to exchange data between core 0 and core 1 for canmore cli debug commands
enum dbg_cmd_type {
    DBG_CMD_SBS_READINT,    // Expects DBG_RESULT_INT
    DBG_CMD_SBS_READBLOCK,  // Expects DBG_RESULT_BLOCK
    DBG_CMD_MFG_READBLOCK,  // Expects DBG_RESULT_BLOCK
    DBG_CMD_MFG_CMD,        // Expects DBG_RESULT_CMD_OK
    DBG_CMD_DF_WRITE        // Expects DBG_RESULT_CMD_OK
};
enum dbg_result_type { DBG_RESULT_ERR, DBG_RESULT_INT, DBG_RESULT_BLOCK, DBG_RESULT_CMD_OK };

static volatile struct core1_dbg_cmd {
    bool in_progress;            // Set by core 0 before req, cleared by core 1 when ready (or core 0 on timeout)
    uint16_t cmd;                // The smbus cmd register value (either mfg or sbs depending on the data)
    enum dbg_cmd_type cmd_type;  // The command to execute on core 1
    union {
        enum read_width int_read_width;  // The read width for the DBG_CMD_SBS_READINT
        uint8_t block_write_count;       // The number of bytes to write with the DBG_CMD_DF_WRITE
    } req_data;
    enum dbg_result_type result_type;
    union {
        // Result depending on result_type value
        // Set by core 1 before clearing in_progress
        uint32_t int_result;
        bq_error_t err_result;
        uint8_t block_read_count;
    } result;
    // Holds contents of blck data to read/write
    uint8_t block_data[32];
} debug_cmd_status;

#define shared_status_is_connected                                                                                     \
    (shared_status.state != BATT_STATE_DISCONNECTED && shared_status.state != BATT_STATE_UNINITIALIZED)

#include "bq40_canmore_cmds.h"

// ===========================================
// Entry point and control loop of core1
// ===========================================
static void __time_critical_func(core1_main)(void) {
    bq_init();

    // Emergency FET Shutdown State Tracking
    enum emshut_state {
        EMSHUT_STATE_NORMAL,
        EMSHUT_STATE_POWER_CYCLE,
        EMSHUT_STATE_LATCH_OFF
    } emshut_state = EMSHUT_STATE_NORMAL;
    // Timeout to wait for the emfet
    bool waiting_for_emshut_flag = false;
    absolute_time_t emshut_seen_timeout = nil_time;
    uint32_t power_cycle_time_ms = 0;
    absolute_time_t power_cycle_timeout = nil_time;

    // Refresh state tracking
    absolute_time_t next_bq40z80_refresh = get_absolute_time();
    uint error_count = 0;

    // Mark as static so we don't eat up our stack
    static bq_mfg_info_t mfg_info;
    static bq_battery_info_t batt_info;

    while (1) {
        safety_core1_checkin();

        if (time_reached(next_bq40z80_refresh)) {
            next_bq40z80_refresh = make_timeout_time_ms(BQ40Z80_REFRESH_TIME_MS);

            // Capture state from last refresh
            bool connected = shared_status_is_connected;

            // Perform refresh of device registers
            bq_error_t err = { .fields.error_code = BQ_ERROR_SUCCESS };
            if (!connected) {
                // If we aren't connected, we need to first start by scanning the mfg info
                err = bq_read_mfg_info(&mfg_info);

                // Report error if this PCB's burned serial number doesn't match the BMS's serial
                if (BQ_CHECK_SUCCESSFUL(err)) {
                    if (mfg_info.serial != sbh_mcu_serial) {
                        safety_raise_fault_with_arg(FAULT_BQ40_MISMATCHED_SERIAL,
                                                    (sbh_mcu_serial << 16) | mfg_info.serial);
                    }
                    else {
                        safety_lower_fault(FAULT_BQ40_MISMATCHED_SERIAL);
                    }
                }
            }

            if (BQ_CHECK_SUCCESSFUL(err)) {
                // Don't scan regular regs of we failed to readout mfg info
                err = bq_read_battery_info(&mfg_info, &batt_info);
            }

            // Handle the data we received
            if (BQ_CHECK_SUCCESSFUL(err)) {
                // Refresh successful do processing
                error_count = 0;

                // If this was the first successful refresh after disconnect, we just connected again
                // Handle the refreshed data
                if (!connected) {
                    safety_lower_fault(FAULT_BQ40_NOT_CONNECTED);

                    // Update shared status
                    uint32_t irq = spin_lock_blocking(shared_status.lock);
                    memcpy((void *) &shared_status.mfg_info, &mfg_info, sizeof(shared_status.mfg_info));
                    spin_unlock(shared_status.lock, irq);
                }

                // Raise fault if safety status or pf status is set
                if (batt_info.safety_status) {
                    safety_raise_fault_with_arg(FAULT_BQ40_SAFETY_STATUS, batt_info.safety_status);
                }
                else {
                    safety_lower_fault(FAULT_BQ40_SAFETY_STATUS);
                }
                if (batt_info.pf_status) {
                    safety_raise_fault_with_arg(FAULT_BQ40_PF_STATUS, batt_info.pf_status);
                }
                else {
                    safety_lower_fault(FAULT_BQ40_PF_STATUS);
                }

                // Update the SOC LEDs
                if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_INIT)) {
                    // Don't show LEDs during init, the SOC is garbage at the time
                    bq_clear_soc_leds();
                }
                else {
                    bq_update_soc_leds(batt_info.relative_soc);
                }

                // Handle emergency FET shutdown logic
                if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_EMSHUT)) {
                    // BQ40 is in emergency fet shutdown, process things accordingly

                    if (emshut_state == EMSHUT_STATE_POWER_CYCLE) {
                        if (waiting_for_emshut_flag) {
                            // First time seeing the flag, we're now in emshut, start the power cycle timer
                            power_cycle_timeout = make_timeout_time_ms(power_cycle_time_ms);
                        }
                        else if (time_reached(power_cycle_timeout)) {
                            // The power cycle is now done, okay to exit emergency fet shutdown
                            bq_error_t err = bq_emshut_exit();
                            if (!BQ_CHECK_SUCCESSFUL(err)) {
                                // Failed to exit emergency fet shutdown, report the error
                                safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err.data);
                            }
                            else {
                                // We exited the power cycle, go back to normal state
                                emshut_state = EMSHUT_STATE_NORMAL;
                            }
                        }
                    }
                    else if (emshut_state == EMSHUT_STATE_LATCH_OFF) {
                        // We are in latch off until we receive a command to go back to normal
                        // Don't do anything
                    }
                    else {
                        // We're in normal state, we must have been told to exit latch off state (or a timeount went
                        // through) - Exit the emshut mode
                        bq_error_t err = bq_emshut_exit();
                        if (!BQ_CHECK_SUCCESSFUL(err)) {
                            // Failed to exit emergency fet shutdown, report the error
                            safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err.data);
                        }
                    }

                    // We know we're not waiting for the emshut flag, since we just saw it
                    waiting_for_emshut_flag = false;
                }
                else {
                    if (waiting_for_emshut_flag && time_reached(emshut_seen_timeout)) {
                        // We were waiting for an emshut flag, but the timeout passed
                        // That means the emshut command failed. Raise a fault
                        // Don't cancel the emshut though, if it eventually goes through logic above will get us out
                        safety_raise_fault(FAULT_BQ40_COMMAND_FAIL);

                        waiting_for_emshut_flag = false;
                        emshut_state = EMSHUT_STATE_NORMAL;
                    }
                }

                // Determine our new current state
                batt_state_t new_state;
                if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_SDM) ||
                    sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_SDV)) {
                    // If the pack is trying to shut down, set state to shutdown
                    new_state = BATT_STATE_NEEDS_SHUTDOWN;
                }
                else if (batt_info.pf_status) {
                    // If we're in permenant fail, that takes highest priority
                    new_state = BATT_STATE_PERMENANT_FAIL;
                }
                else if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_INIT)) {
                    // If gauge is initializing, the reported data will be garbage, don't show to the screen
                    new_state = BATT_STATE_INITIALIZING;
                }
                // Emergency FET Shutdown
                else if (emshut_state == EMSHUT_STATE_POWER_CYCLE && !waiting_for_emshut_flag) {
                    new_state = BATT_STATE_POWER_CYCLE;
                }
                else if (emshut_state == EMSHUT_STATE_LATCH_OFF && !waiting_for_emshut_flag) {
                    new_state = BATT_STATE_LATCH_OFF;
                }
                else if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_PRES)) {
                    // Battery is plugged in, see what is happening right now
                    if (!sbs_check_bit(batt_info.battery_status, SBS_BATTERY_STATUS_DSG)) {
                        // Battery is charging of BatteryStatus()[DSG] is cleared
                        // This takes priority over XDSG as this means that the charge current is greater than
                        // Chg Current Threshold
                        // This is only possible if there's power on the PACK+ pin charging the battery
                        new_state = BATT_STATE_CHARGING;
                    }
                    else if (sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_XDSG) ||
                             !sbs_check_bit(batt_info.operation_status, SBS_OPERATION_STATUS_DSG)) {
                        // Discharging is disabled for whatever reason, and we aren't charging right now
                        // This is an issue since we should have PRES set so we should be allowed to discharge
                        // (That's what someone who uses the battery will expect, the cable is plugged in)
                        // Report that this is a problem (and not to trust the side detect pin)
                        new_state = BATT_STATE_XDSG;
                    }
                    else {
                        // Battery is plugged in, we're discharging
                        new_state = BATT_STATE_DISCHARGING;
                    }
                }
                else {
                    // Pack isn't plugged in, but ready to do things, report pack is removed
                    new_state = BATT_STATE_REMOVED;
                }

                // Refresh the shared battery info
                uint32_t irq = spin_lock_blocking(shared_status.lock);
                shared_status.state = new_state;
                memcpy((void *) &shared_status.batt_info, &batt_info, sizeof(shared_status.batt_info));
                spin_unlock(shared_status.lock, irq);
            }
            else {
                if (shared_status.state != BATT_STATE_DISCONNECTED && error_count < BQ_MAX_ERRORS_BEFORE_DISCONNECT) {
                    error_count++;
                }
                else {
                    safety_raise_fault_with_arg(FAULT_BQ40_NOT_CONNECTED, err.data);
                    bq_clear_soc_leds();

                    if (shared_status.state != BATT_STATE_NEEDS_SHUTDOWN) {
                        // Set state to disconnected and wake it up again only if it didn't need to shut down
                        // If not, then this will keep waking the bq40 up even when it's trying to shut off

                        // No need to lock, word writes are atomic
                        shared_status.state = BATT_STATE_DISCONNECTED;

                        // Try to wake up the battery in case its in shutdown
                        bq_pulse_wake();
                    }
                }
            }
        }

        // Handle fifo affairs
        while (multicore_fifo_rvalid()) {
            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };

            batt_state_t state = shared_status.state;

            // Handle side detect read
            if (req.type == FIFO_CMD_READ_SIDE_DETECT) {
                if (state == BATT_STATE_CHARGING || state == BATT_STATE_DISCHARGING) {
                    // The PACK+ should be valid, it's okay to read the side detect
                    bool side_det;
                    bq_error_t err = bq_read_side_detect(&side_det);

                    // Process the results
                    if (BQ_CHECK_SUCCESSFUL(err)) {
                        // Successfully read side detect, send the result back to the other core
                        uint32_t irq = spin_lock_blocking(shared_status.lock);
                        shared_status.req_results.side_detect = side_det;
                        shared_status.req_results.side_detect_read_state = READ_STATE_UNREAD;
                        spin_unlock(shared_status.lock, irq);
                    }
                    else {
                        // Read failed, report the error and go back to idle state
                        safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err.data);
                        shared_status.req_results.side_detect_read_state = READ_STATE_IDLE;
                    }
                }
                else {
                    // The PACK+ wouldn't be valid in this state, fail the command
                    safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, state);
                    shared_status.req_results.side_detect_read_state = READ_STATE_IDLE;
                }
            }

            // Handle register reads
            else if (req.type == FIFO_CMD_READ_GAUGING_STATUS) {
                bq_error_t err = bq_read_gauging_status((uint32_t *) (&shared_status.req_results.reg_read_result));
                if (BQ_CHECK_SUCCESSFUL(err)) {
                    shared_status.req_results.reg_read_okay = true;
                }
                else {
                    shared_status.req_results.reg_read_okay = false;
                    safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err.data);
                }
                shared_status.req_results.reg_read_pending = false;
            }
            else if (req.type == FIFO_CMD_READ_CHARGING_STATUS) {
                bq_error_t err = bq_read_charging_status((uint32_t *) (&shared_status.req_results.reg_read_result));
                if (BQ_CHECK_SUCCESSFUL(err)) {
                    shared_status.req_results.reg_read_okay = true;
                }
                else {
                    shared_status.req_results.reg_read_okay = false;
                    safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err.data);
                }
                shared_status.req_results.reg_read_pending = false;
            }

            // Handle Emergency FET Commands
            else if (req.type == FIFO_CMD_POWER_CYCLE || req.type == FIFO_CMD_KILL_POWER) {
                if ((state == BATT_STATE_CHARGING || state == BATT_STATE_DISCHARGING) &&
                    emshut_state == EMSHUT_STATE_NORMAL) {
                    // Only enter power cycle if we're plugged in and we're not doing an emshut already

                    // Start the new emshut command
                    emshut_seen_timeout = make_timeout_time_ms(EMSHUT_STATE_TIMEOUT);
                    emshut_state =
                        (req.type == FIFO_CMD_POWER_CYCLE ? EMSHUT_STATE_POWER_CYCLE : EMSHUT_STATE_LATCH_OFF);
                    waiting_for_emshut_flag = true;

                    // Set the power cycle duration from the argument if this is a power cycle command
                    if (req.type == FIFO_CMD_POWER_CYCLE) {
                        power_cycle_time_ms = req.arg;
                    }
                }
                else {
                    // Can't do this command right now
                    safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, (state << 16) | emshut_state);
                }
            }

            // Handle debug commands
            else if (req.type == FIFO_CMD_DBG_COMMAND) {
                uint32_t irq = spin_lock_blocking(shared_status.lock);
                if (!debug_cmd_status.in_progress) {
                    // Command got aborted before we got to it, don't process it
                    spin_unlock(shared_status.lock, irq);
                }
                else {
                    enum dbg_cmd_type cmd_type = debug_cmd_status.cmd_type;
                    uint16_t cmd = debug_cmd_status.cmd;
                    enum read_width width = debug_cmd_status.req_data.int_read_width;
                    spin_unlock(shared_status.lock, irq);

                    bq_error_t err = { .fields = { .error_code = BQ_ERROR_BAD_DBG_COMMAND, .arg = 0, .line = 0 } };

                    if (cmd_type == DBG_CMD_SBS_READINT) {
                        uint32_t out;
                        err = bq_dbg_read_sbs_int(cmd, width, &out);

                        // Return result on success
                        if (BQ_CHECK_SUCCESSFUL(err)) {
                            irq = spin_lock_blocking(shared_status.lock);
                            if (debug_cmd_status.in_progress) {
                                // Only write if command didn't get aborted
                                debug_cmd_status.result_type = DBG_RESULT_INT;
                                debug_cmd_status.result.int_result = out;
                                debug_cmd_status.in_progress = false;
                            }
                            spin_unlock(shared_status.lock, irq);
                        }
                    }
                    else if (cmd_type == DBG_CMD_SBS_READBLOCK) {
                        size_t len = sizeof(debug_cmd_status.block_data);
                        err = bq_dbg_read_sbs_block(cmd, (uint8_t *) debug_cmd_status.block_data, &len);

                        // Return result on success
                        if (BQ_CHECK_SUCCESSFUL(err)) {
                            irq = spin_lock_blocking(shared_status.lock);
                            if (debug_cmd_status.in_progress) {
                                // Only write if command didn't get aborted
                                debug_cmd_status.result_type = DBG_RESULT_BLOCK;
                                debug_cmd_status.result.block_read_count = len;
                                debug_cmd_status.in_progress = false;
                            }
                            spin_unlock(shared_status.lock, irq);
                        }
                    }
                    else if (cmd_type == DBG_CMD_MFG_READBLOCK) {
                        size_t len = sizeof(debug_cmd_status.block_data);
                        err = bq_dbg_read_mfg_block(cmd, (uint8_t *) debug_cmd_status.block_data, &len);

                        // Return result on success
                        if (BQ_CHECK_SUCCESSFUL(err)) {
                            irq = spin_lock_blocking(shared_status.lock);
                            if (debug_cmd_status.in_progress) {
                                // Only write if command didn't get aborted
                                debug_cmd_status.result_type = DBG_RESULT_BLOCK;
                                debug_cmd_status.result.block_read_count = len;
                                debug_cmd_status.in_progress = false;
                            }
                            spin_unlock(shared_status.lock, irq);
                        }
                    }
                    else if (cmd_type == DBG_CMD_MFG_CMD) {
                        err = bq_dbg_mfg_cmd(cmd);

                        // Return result on success
                        if (BQ_CHECK_SUCCESSFUL(err)) {
                            irq = spin_lock_blocking(shared_status.lock);
                            if (debug_cmd_status.in_progress) {
                                // Only write if command didn't get aborted
                                debug_cmd_status.result_type = DBG_RESULT_CMD_OK;
                                debug_cmd_status.in_progress = false;
                            }
                            spin_unlock(shared_status.lock, irq);
                        }
                    }
                    else if (cmd_type == DBG_CMD_DF_WRITE) {
                        err = bq_dbg_df_write(cmd, (uint8_t *) debug_cmd_status.block_data,
                                              debug_cmd_status.req_data.block_write_count);

                        // Return result on success
                        if (BQ_CHECK_SUCCESSFUL(err)) {
                            irq = spin_lock_blocking(shared_status.lock);
                            if (debug_cmd_status.in_progress) {
                                // Only write if command didn't get aborted
                                debug_cmd_status.result_type = DBG_RESULT_CMD_OK;
                                debug_cmd_status.in_progress = false;
                            }
                            spin_unlock(shared_status.lock, irq);
                        }
                    }

                    if (!BQ_CHECK_SUCCESSFUL(err)) {
                        irq = spin_lock_blocking(shared_status.lock);
                        if (debug_cmd_status.in_progress) {
                            // Only write if command didn't get aborted
                            debug_cmd_status.result_type = DBG_RESULT_ERR;
                            debug_cmd_status.result.err_result = err;
                            debug_cmd_status.in_progress = false;
                        }
                        spin_unlock(shared_status.lock, irq);
                    }
                }
            }
            else {
                // Unrecognized command, report error
                safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, req.raw);
            }
        }
    }
}

// ============================================
// Exported Function for core0
// ============================================

void core1_init(uint8_t expected_serial) {
    shared_status.lock = spin_lock_init(spin_lock_claim_unused(true));
    shared_status.lock = spin_lock_init(spin_lock_claim_unused(true));

    core1_register_canmore_cmds();

    sbh_mcu_serial = expected_serial;
    safety_launch_core1(core1_main);
}

batt_state_t core1_get_batt_state(void) {
    // No need to lock since it's atomic
    return shared_status.state;
}

bool core1_get_pack_mfg_info(bq_mfg_info_t *pack_info_out) {
    bool read_successful = false;

    uint32_t irq = spin_lock_blocking(shared_status.lock);
    if (shared_status_is_connected) {
        memcpy(pack_info_out, (void *) &shared_status.mfg_info, sizeof(shared_status.mfg_info));
        read_successful = true;
    }
    spin_unlock(shared_status.lock, irq);

    return read_successful;
}

// ============================================
// Exported State Query Functions
// ============================================

int32_t core1_avg_current(void) {
    return shared_status.batt_info.avg_current;
}

uint8_t core1_soc(void) {
    return shared_status.batt_info.relative_soc;
}

uint16_t core1_time_remaining(bool *is_charging) {
    uint16_t remaining;

    uint32_t irq = spin_lock_blocking(shared_status.lock);
    if (sbs_check_bit(shared_status.batt_info.battery_status, SBS_BATTERY_STATUS_DSG)) {
        *is_charging = false;
        remaining = shared_status.batt_info.time_to_empty;
    }
    else {
        *is_charging = true;
        remaining = shared_status.batt_info.time_to_full;
    }
    spin_unlock(shared_status.lock, irq);

    return remaining;
}

uint16_t core1_voltage(void) {
    return shared_status.batt_info.da_status1.batt_voltage;
}

int32_t core1_current(void) {
    return shared_status.batt_info.current;
}

void core1_cell_voltages(uint16_t *cell_voltages_out) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    cell_voltages_out[0] = shared_status.batt_info.da_status1.cell1_voltage;
    cell_voltages_out[1] = shared_status.batt_info.da_status1.cell2_voltage;
    cell_voltages_out[2] = shared_status.batt_info.da_status1.cell3_voltage;
    cell_voltages_out[3] = shared_status.batt_info.da_status1.cell4_voltage;
    cell_voltages_out[4] = shared_status.batt_info.cell5_voltage;
    spin_unlock(shared_status.lock, irq);
}

uint16_t core1_batt_temp(void) {
    return shared_status.batt_info.temperature;
}

// ============================================
// Exported Command Functions
// ============================================

bool core1_get_side_detect(bool *side_det_is_high) {
    sio_fifo_req_t req = {};
    bool copied = false;

    // Handles the various read state transitions
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    switch (shared_status.req_results.side_detect_read_state) {
    case READ_STATE_IDLE:
    default:
        // No command in progress, send new read request
        shared_status.req_results.side_detect_read_state = READ_STATE_IN_PROGRESS;
        req.type = FIFO_CMD_READ_SIDE_DETECT;
        multicore_fifo_push_blocking(req.raw);
        break;

    case READ_STATE_IN_PROGRESS:
        // Still in progress, do nothing
        break;

    case READ_STATE_UNREAD:
        // Data is unread, copy it out
        copied = true;
        *side_det_is_high = shared_status.req_results.side_detect;
        shared_status.req_results.side_detect_read_state = READ_STATE_IDLE;
        break;
    }
    spin_unlock(shared_status.lock, irq);

    return copied;
}

void core1_power_cycle_robot(void) {
    sio_fifo_req_t req = { .type = FIFO_CMD_POWER_CYCLE, .arg = ROBOT_POWER_CYCLE_TIME_MS };
    multicore_fifo_push_blocking(req.raw);
}

void core1_kill_robot_power(void) {
    sio_fifo_req_t req = { .type = FIFO_CMD_KILL_POWER, .arg = 0 };
    multicore_fifo_push_blocking(req.raw);
}

bool core1_get_gauging_status(uint32_t *status_out) {
    hard_assert(!shared_status.req_results.reg_read_pending);
    shared_status.req_results.reg_read_pending = true;

    sio_fifo_req_t req = { .type = FIFO_CMD_READ_GAUGING_STATUS, .arg = ROBOT_POWER_CYCLE_TIME_MS };
    multicore_fifo_push_blocking(req.raw);

    while (shared_status.req_results.reg_read_pending)
        tight_loop_contents();

    if (shared_status.req_results.reg_read_okay) {
        *status_out = shared_status.req_results.reg_read_result;
        return true;
    }
    else {
        return false;
    }
}

bool core1_get_charging_status(uint32_t *status_out) {
    hard_assert(!shared_status.req_results.reg_read_pending);
    shared_status.req_results.reg_read_pending = true;

    sio_fifo_req_t req = { .type = FIFO_CMD_READ_CHARGING_STATUS, .arg = ROBOT_POWER_CYCLE_TIME_MS };
    multicore_fifo_push_blocking(req.raw);

    while (shared_status.req_results.reg_read_pending)
        tight_loop_contents();

    if (shared_status.req_results.reg_read_okay) {
        *status_out = shared_status.req_results.reg_read_result;
        return true;
    }
    else {
        return false;
    }
}
