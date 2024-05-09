#define RUNNING_ON_CORE1
#include "core1.h"

#include "bq40z80.h"
#include "safety_interface.h"

#include "pico/multicore.h"
#include "titan/logger.h"

#include <stdio.h>
#include <string.h>

#define BQ40Z80_REFRESH_TIME_MS 200
#define FIFO_REQ_VALUE_WIDTH 24

static uint8_t sbh_mcu_serial;

typedef union sio_fifo_req {
    struct __packed {
        enum __packed cmd_type {
            BQ_POWER_CYCLE,
            BQ_KILL_POWER,
            BQ_READ_CYCLE_COUNT,
            BQ_READ_STATE_OF_HEALTH,
            BQ_READ_CAPACITY
        } type;
        uint32_t arg : FIFO_REQ_VALUE_WIDTH;
    };
    uint32_t raw;
} sio_fifo_req_t;

static_assert(sizeof(sio_fifo_req_t) == sizeof(uint32_t), "FIFO Command did not pack properly");

// ================================================
// Shared Memory Definitions
// ================================================
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

    bool connected;
    struct bq_battery_info_t batt_info;
    struct bq_mfg_info_t mfg_info;

} shared_status = { 0 };

// ===========================================
// Entry point and control loop of core1
// ===========================================
static void __time_critical_func(core1_main)(void) {
    bq_init();

    absolute_time_t next_bq40z80_refresh = get_absolute_time();
    uint error_count = 0;

    // Mark as static so we don't eat up our stack
    static bq_mfg_info_t mfg_info;
    static bq_battery_info_t batt_info;

    while (1) {
        safety_core1_checkin();

        if (time_reached(next_bq40z80_refresh)) {
            next_bq40z80_refresh = make_timeout_time_ms(BQ40Z80_REFRESH_TIME_MS);

            // Perform refresh of device registers
            bq_error_t err = { .fields.error_code = BQ_ERROR_SUCCESS };
            if (!shared_status.connected) {
                // If we aren't connected, we need to first start by scanning the mfg info
                err = bq_read_mfg_info(&mfg_info);

                // Report error if this PCB's burned serial number doesn't match the BMS's serial/
                if (mfg_info.serial != sbh_mcu_serial) {
                    safety_raise_fault_with_arg(FAULT_BQ40_MISMATCHED_SERIAL, (sbh_mcu_serial << 16) | mfg_info.serial);
                }
                else {
                    safety_lower_fault(FAULT_BQ40_MISMATCHED_SERIAL);
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
                if (!shared_status.connected) {
                    safety_lower_fault(FAULT_BQ40_NOT_CONNECTED);

                    // Update shared status
                    uint32_t irq = spin_lock_blocking(shared_status.lock);
                    memcpy((void *) &shared_status.mfg_info, &mfg_info, sizeof(shared_status.mfg_info));
                    shared_status.connected = true;
                    spin_unlock(shared_status.lock, irq);
                }

                // Refresh the battery info
                bq_update_soc_leds(batt_info.relative_soc);
                uint32_t irq = spin_lock_blocking(shared_status.lock);
                memcpy((void *) &shared_status.batt_info, &batt_info, sizeof(shared_status.batt_info));
                spin_unlock(shared_status.lock, irq);
            }
            else {
                if (shared_status.connected) {
                    error_count++;
                    if (error_count > BQ_MAX_ERRORS_BEFORE_DISCONNECT) {
                        // No need to lock, word writes are atomic
                        shared_status.connected = false;

                        safety_raise_fault_with_arg(FAULT_BQ40_NOT_CONNECTED, err.data);
                    }
                }
                else {
                    safety_raise_fault_with_arg(FAULT_BQ40_NOT_CONNECTED, err.data);
                    // Try to wake up the battery
                    bq_pulse_wake();
                }
            }
        }

        // Handle fifo affairs
        while (multicore_fifo_rvalid()) {
            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };
            if (shared_status.connected) {
                // if (req.type == BQ_POWER_CYCLE) {
                //     bq_open_dschg_temp(req.arg);  // Command is fet open time milliseconds
                // }
                // TODO still WIP for pack chemistry, cycle count and stateofhealth
            }
            else {
                // Drop the command if we don't support it
                // Raise fault to show that we couldn't send it
                safety_raise_fault(FAULT_BQ40_COMMAND_FAIL);
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

    sbh_mcu_serial = expected_serial;
    safety_launch_core1(core1_main);
}

bool core1_get_pack_mfg_info(bq_mfg_info_t *pack_info_out) {
    bool read_successful = false;

    uint32_t irq = spin_lock_blocking(shared_status.lock);
    if (shared_status.connected) {
        memcpy(pack_info_out, (void *) &shared_status.mfg_info, sizeof(shared_status.mfg_info));
        read_successful = true;
    }
    spin_unlock(shared_status.lock, irq);

    return read_successful;
}

bool core1_check_present(void) {
    // No need to worry about locking, since the only race is that connected will be true and then operation_status will
    // be invalid after connected turns to false. However, this will never be the case, and in the event connected does
    // change before reading pres_flag, it changes nothing since operation_status won't get touched
    if (!shared_status.connected) {
        return false;
    }
    return sbs_check_bit(shared_status.batt_info.operation_status, SBS_OPERATION_STATUS_PRES);
}

bool core1_check_port_detected(void) {
    // TODO: We need to figure out how this will be negotiated, this won't be through refresh

    return shared_status.batt_info.port_detected;
}

bool core1_dsg_mode(void) {
    return sbs_check_bit(shared_status.batt_info.battery_status, SBS_BATTERY_STATUS_DSG);
}

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
    return shared_status.batt_info.da_status1.pack_voltage;
}

int32_t core1_current(void) {
    return shared_status.batt_info.current;
}

void core1_open_dsg_temp(const uint32_t open_time_ms) {
    sio_fifo_req_t req = { .type = BQ_POWER_CYCLE, .arg = open_time_ms };
    multicore_fifo_push_blocking(req.raw);
}
