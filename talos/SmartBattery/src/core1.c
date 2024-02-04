#include "core1.h"

#include "bq40z80.h"
#include "pio_i2c.h"
#include "safety_interface.h"

#include "pico/multicore.h"
#include "titan/logger.h"

#include <stdio.h>

#define BQ40Z80_REFRESH_TIME_MS 100
#define FIFO_REQ_VALUE_WIDTH 24

static uint32_t fet_open_time_ms;
static bool read_once_cached = false;
static uint8_t sbh_mcu_serial;
static bq_mfg_info_t battery_mfg_info;
static bq_battery_info_t battery_status_info;

static absolute_time_t next_bq40z80_refresh;

typedef union sio_fifo_req {
    struct __packed {
        enum __packed cmd_type {
            BQ_POWER_CYCLE,
            BQ_CYCLE_COUNT,
            BQ_CHEMISTRY,
            BQ_STATE_OF_HEALTH,
            // TODO might add more
            // BQ_MAC_REG_ADDR,
            // BQ_MAC_RESET_CMD,
            // BQ_MAC_SHTDN_CMD,
        } type;
        uint32_t cmd : FIFO_REQ_VALUE_WIDTH;
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

    bool pres_flag;
    bool port_detected;
    int16_t avg_current;
    uint8_t soc;
    bool dsg_mode;
    uint16_t time_to_empty;
    uint16_t time_to_full;
    uint16_t voltage;
    int16_t current;
    uint16_t chg_voltage;
    uint16_t chg_current;

} shared_status = { 0 };

static volatile struct core1_mfg_info_shared_mem {
    /**
     * @brief spin lock to protect manufacturer info transferring across cores
     *
     */
    spin_lock_t *lock;

    /**
     * @brief manufacturer info from bq40z80 including:
     * device name, device date, device serial number
     *
     */
    struct bq_mfg_info_t pack_info;
} shared_mfg_info = { 0 };

static void core1_bq40z80_error_cb(const bq_error type, const int error_code) {
    if (type == BQ_ERROR_SAFETY_STATUS) {
        safety_raise_fault_with_arg(FAULT_BQ40_SAFETY_STATUS, error_code);
    }
    else {
        safety_raise_fault_with_arg(FAULT_BQ40_ERROR, type);
    }
}

/**
 * @brief flush data to shared_mfg_info via locking spin lock
 *
 * @param pack_info
 */
static void core1_bq40z80_flush_mfg_info(bq_mfg_info_t *pack_info) {
    uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
    shared_mfg_info.pack_info.serial = pack_info->serial;
    shared_mfg_info.pack_info.mfg_day = pack_info->mfg_day;
    shared_mfg_info.pack_info.mfg_mo = pack_info->mfg_mo;
    shared_mfg_info.pack_info.mfg_year = pack_info->mfg_year;
    for (uint8_t i = 0; i < 21; i++) {
        shared_mfg_info.pack_info.name[i] = pack_info->name[i];
        if (shared_mfg_info.pack_info.name[i] == 0)
            break;
    }
    spin_unlock(shared_mfg_info.lock, irq);
}

/**
 * @brief flush data to shared_status via locking spin lock
 *
 * @param bat_stat
 */
static void core1_bq40z80_flush_battery_info(bq_battery_info_t *bat_stat) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    shared_status.pres_flag = bat_stat->battery_presence;
    shared_status.port_detected = bat_stat->port_detected;
    shared_status.avg_current = bat_stat->avg_current;
    shared_status.soc = bat_stat->soc;
    shared_status.dsg_mode = bat_stat->dsg_mode;
    shared_status.time_to_empty = bat_stat->time_to_empty;
    shared_status.voltage = bat_stat->voltage;
    shared_status.current = bat_stat->current;
    shared_status.time_to_full = bat_stat->time_to_full;
    shared_status.chg_voltage = bat_stat->chg_voltage;
    shared_status.chg_current = bat_stat->chg_current;

    spin_unlock(shared_status.lock, irq);
}

// ===========================================
// Entry point and control loop of core1
// ===========================================
static void __time_critical_func(core1_main)(void) {
    bq40z80_init(&core1_bq40z80_error_cb);

    while (1) {
        safety_core1_checkin();

        if (time_reached(next_bq40z80_refresh)) {
            next_bq40z80_refresh = make_timeout_time_ms(BQ40Z80_REFRESH_TIME_MS);
            if (bq40z80_refresh_reg(sbh_mcu_serial, read_once_cached, &battery_status_info, &battery_mfg_info)) {
                bq40z80_update_soc_leds(core1_soc());
                if (!read_once_cached) {
                    read_once_cached = true;
                    core1_bq40z80_flush_mfg_info(&battery_mfg_info);
                }
                core1_bq40z80_flush_battery_info(&battery_status_info);
            }
        }

        // Handle fifo affairs
        while (multicore_fifo_rvalid()) {
            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };
            if (req.type == BQ_POWER_CYCLE) {
                bq_open_dschg_temp(fet_open_time_ms);
            }
            // TODO still WIP for pack chemistry, cycle count and stateofhealth
        }
    }
}

// ============================================
// Exported Function for core0
// ============================================

void core1_init(uint8_t expected_serial) {
    shared_status.lock = spin_lock_init(spin_lock_claim_unused(true));
    shared_mfg_info.lock = spin_lock_init(spin_lock_claim_unused(true));

    sbh_mcu_serial = expected_serial;
    safety_launch_core1(core1_main);
}

bool core1_get_pack_mfg_info(bq_mfg_info_t *pack_info_out) {
    bool read_successful = false;
    if (read_once_cached) {
        uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
        pack_info_out->serial = shared_mfg_info.pack_info.serial;
        pack_info_out->mfg_day = shared_mfg_info.pack_info.mfg_day;
        pack_info_out->mfg_mo = shared_mfg_info.pack_info.mfg_mo;
        pack_info_out->mfg_year = shared_mfg_info.pack_info.mfg_year;

        for (uint8_t i = 0; i < 21; i++) {
            pack_info_out->name[i] = shared_mfg_info.pack_info.name[i];
            if (pack_info_out->name[i] == 0)
                break;
        }

        spin_unlock(shared_mfg_info.lock, irq);
        read_successful = true;
    }
    return read_successful;
}

bool core1_check_present(void) {
    bool present_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    present_out = shared_status.pres_flag;
    spin_unlock(shared_status.lock, irq);
    return present_out;
}

bool core1_check_port_detected(void) {
    bool port_detected_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    port_detected_out = shared_status.port_detected;
    spin_unlock(shared_status.lock, irq);
    return port_detected_out;
}

bool core1_dsg_mode(void) {
    bool dsg_mode_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    dsg_mode_out = shared_status.dsg_mode;
    spin_unlock(shared_status.lock, irq);
    return dsg_mode_out;
}

int16_t core1_avg_current(void) {
    int16_t avg_curr_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    avg_curr_out = shared_status.avg_current;
    spin_unlock(shared_status.lock, irq);
    return avg_curr_out;
}

uint8_t core1_soc(void) {
    uint8_t soc_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    soc_out = shared_status.soc;
    spin_unlock(shared_status.lock, irq);
    return soc_out;
}

uint16_t core1_time_to_empty(void) {
    uint16_t remaining_time_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    remaining_time_out = shared_status.time_to_empty;
    spin_unlock(shared_status.lock, irq);
    return remaining_time_out;
}

uint16_t core1_voltage(void) {
    uint16_t voltage_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    voltage_out = shared_status.voltage;
    spin_unlock(shared_status.lock, irq);
    return voltage_out;
}

int16_t core1_current(void) {
    int16_t current_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    current_out = shared_status.current;
    spin_unlock(shared_status.lock, irq);
    return current_out;
}

void core1_open_dsg_temp(const uint32_t open_time_ms) {
    fet_open_time_ms = open_time_ms;
    sio_fifo_req_t req = { .type = BQ_POWER_CYCLE, .cmd = 0x270C };
    multicore_fifo_push_blocking(req.raw);
}
