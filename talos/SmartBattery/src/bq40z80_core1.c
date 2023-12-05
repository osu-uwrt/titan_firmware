#include "bq40z80_core1.h"

#include "pio_i2c.h"
#include "safety_interface.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "titan/logger.h"

#include <stdio.h>

#define RETRANSMIT_COUNT 4
#define START_UP_TIME_EXPIRED_MS 5000
#define FIFO_REQ_VALUE_WIDTH 24

uint8_t BQ_LEDS_CORE1[3] = { LED_R_PIN, LED_Y_PIN, LED_G_PIN };
uint pio_i2c_program;
static uint32_t fet_open_time_ms;
static bool connected = false;
static bool read_once_cached = false;
static absolute_time_t start_up_expired;
static uint8_t sbh_mcu_serial;

typedef union sio_fifo_req {
    struct __packed {
        enum __packed cmd_type {
            BQ_MAC_EMG_FET_CTRL_CMD,
            BQ_MAC_EMG_FET_OFF_CMD,
            BQ_MAC_EMG_FET_ON_CMD
            // TODO might add more
            // BQ_MAC_REG_ADDR = 0x0000,
            // BQ_MAC_RESET_CMD = 0x0041,
            // BQ_MAC_SHTDN_CMD = 0x0010,
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

    /**
     * @brief basic battery status from bq40z80 including:
     * voltage, current, avg_current, time_to_empty, soc, safety_status_reg, port_detected
     *
     */
    struct core1_battery_status status;

    /**
     * @brief flag whether the battery is present
     *
     */
    bool pres_flag;

    /**
     * @brief flag whether any safety status bit is raised
     *
     */
    bool safety_status_flag;  // FIXME remove from shared_status
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
    struct mfg_info pack_info;
} shared_mfg_info = { 0 };

// ============================================
// I2C Functions
// ============================================
/**
 * @brief perform i2c pio transfer
 *
 * @param bq_reg bq command
 * @param rx_buf output buffer
 * @param len length of the rx_buf
 * @return int return code of i2c transfer, 0 as successful, otherwise failed
 */
static int bq_handle_i2c_transfer(uint8_t *bq_reg, uint8_t *rx_buf, uint len) {
    int ret_code = 0;
    // slow down the transfers, smbus and pio no likey :/
    sleep_ms(1);
    ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, bq_reg, 1);
    ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, rx_buf, len);
    return ret_code;
}

/**
 * @brief perform i2c pio write
 *
 * @param tx_buf input buffer which is typically bq command
 * @param len length of the tx_buf
 * @return int return code of i2c transfer, 0 as successful, otherwise failed
 */
static int bq_write_only_transfer(uint8_t *tx_buf, uint len) {
    int ret_code = 0;
    // slow down the transfers, smbus and pio no likey :/
    sleep_ms(1);
    ret_code = pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, tx_buf, len);
    return ret_code;
}
/**
 * @brief format command before sending them to bq_write_only_transfer
 *
 * @param command
 * @return int return code of i2c transfer, 0 as successful, otherwise failed
 */
static int send_mac_command(uint16_t command) {
    uint8_t data[3] = {
        0x00,                        // MAC register address (actually 1 byte not 2 so cut off the upper byte)
        (uint8_t) (command >> 8),    // the high byte of the command
        (uint8_t) (command & 0xFF),  // the low byte of the command
    };
    return bq_write_only_transfer(data, 3);
}

// ============================================
// BQ40Z80 Functions
// ============================================

static void bq_error(const core1_bq_error_code type, const int error_code) {
    if (type == BQ_ERROR_I2C_TRANSACTION)
        LOG_FATAL("I2C transfer error: %d", error_code);
    else if (type == BQ_ERROR_SAFETY_STATUS)
        LOG_FATAL("Safety Status register: %d", error_code);
    safety_raise_fault_with_arg(FAULT_BQ40_ERROR, type);
}

static int sbs_read_u1(uint8_t *data, uint8_t cmd) {
    uint8_t rx_buf[1] = { 0 };
    uint8_t bq_cmd[1] = { cmd };
    int ret_code = 0;
    ret_code |= bq_handle_i2c_transfer(bq_cmd, rx_buf, 1);
    *data = rx_buf[0];
    return ret_code;
}

static int sbs_read_i1(int8_t *data, uint8_t cmd) {
    return sbs_read_u1((uint8_t *) data, cmd);
}

static int sbs_read_u2(uint16_t *data, uint8_t cmd) {
    uint8_t rx_buf[2] = { 0, 0 };
    uint8_t bq_cmd[1] = { cmd };
    int ret_code = 0;
    ret_code |= bq_handle_i2c_transfer(bq_cmd, rx_buf, 2);
    *data = rx_buf[1] << 8 | rx_buf[0];
    return ret_code;
}

static int sbs_read_i2(int16_t *data, uint8_t cmd) {
    return sbs_read_u2((uint16_t *) data, cmd);
}

static int sbs_read_h4(uint32_t *data, uint8_t cmd) {
    uint8_t rx_buf[5] = { 0, 0, 0, 0, 0 };
    uint8_t bq_cmd[1] = { cmd };
    int ret_code = 0;
    ret_code |= bq_handle_i2c_transfer(bq_cmd, rx_buf, 5);
    // the 0th byte is length of rx_buf
    if (rx_buf[0] != 4) {
        return -1;
    }
    *data = rx_buf[4] << 24 | rx_buf[3] << 16 | rx_buf[2] << 8 | rx_buf[1];
    return ret_code;
}

/**
 * @brief initialize PIO hardware for i2c usage, GPIO for BQ_LEDS_CORE1, and BMS_WAKE_PIN
 *
 */
static void bq_core1_init() {
    // Init the wake pin, active high
    gpio_init(BMS_WAKE_PIN);
    gpio_set_dir(BMS_WAKE_PIN, GPIO_OUT);
    gpio_put(BMS_WAKE_PIN, 0);

    for (uint8_t led = 0; led < 3; led++) {
        gpio_init(BQ_LEDS_CORE1[led]);
        gpio_set_dir(BQ_LEDS_CORE1[led], GPIO_OUT);
        gpio_put(BQ_LEDS_CORE1[led], 0);
    }

    // init PIO I2C
    pio_i2c_program = pio_add_program(pio0, &i2c_program);
    pio_sm_claim(pio0, PIO_SM);
    i2c_program_init(pio0, PIO_SM, pio_i2c_program, BMS_SDA_PIN, BMS_SCL_PIN);
}

/**
 * @brief try to connect to bq40z80 by checking if the serial number from i2c transfer matched sbh_mcu_serial
 *
 * @return true connect successfully
 * @return false failed to connect, that is start up, for a duration of START_UP_TIME_EXPIRED_MS
 */
static bool try_connect(void) {
    start_up_expired =
        make_timeout_time_ms(START_UP_TIME_EXPIRED_MS);  // FIXME not sure placed before main loop or in here?
    uint16_t serial;
    bool start_up_success = true;
    while (sbs_read_u2(&serial, BQ_READ_CELL_SERI) || serial != sbh_mcu_serial) {
        gpio_put(BMS_WAKE_PIN, 1);
        sleep_ms(1000);
        gpio_put(BMS_WAKE_PIN, 0);
        if (time_reached(start_up_expired)) {
            start_up_success = false;
            break;
        }
    }
    return start_up_success;
}

/**
 * @brief read bq registers (voltage, current, avg_current, soc, time_to_empty, port_detected, safety_status, present)
 * and pass them to output variables bat_out and opr_out
 *
 * @param bat_out battery status output variable
 * @param opr_out operation status output variable
 * @return int return code of i2c transfer, 0 as successful, otherwise failed
 */
static int bq_core1_read_dsg_registers(struct core1_battery_status *bat_out, struct core1_operation_status *opr_out) {
    int ret_code = 0;
    uint16_t gpio_read;
    uint32_t op_status;
    // TODO find a way to find which i2c read transfer fails among these transfers
    ret_code |= sbs_read_u2(&bat_out->voltage, BQ_READ_PACK_VOLT);
    ret_code |= sbs_read_i2(&bat_out->current, BQ_READ_PACK_CURR);
    ret_code |= sbs_read_i2(&bat_out->avg_current, BQ_READ_AVRG_CURR);
    ret_code |= sbs_read_u2(&bat_out->time_to_empty, BQ_READ_TIME_EMPT);
    ret_code |= sbs_read_u1(&bat_out->soc, BQ_READ_RELAT_SOC);
    ret_code |= sbs_read_h4(&op_status, BQ_READ_OPER_STAT);
    ret_code |= sbs_read_u2(&gpio_read, BQ_READ_GPIO);
    bat_out->port_detected = (gpio_read & 0x8) != 0;
    opr_out->present = (op_status & 0x1) != 0;
    if ((opr_out->safety_status = (op_status & 0x800) != 0)) {
        // Raise fault for Safety Status
        uint32_t safety_status_reg;
        ret_code |= sbs_read_h4(&safety_status_reg, BQ_READ_SAFE_STAT);
        bq_error(BQ_ERROR_SAFETY_STATUS, safety_status_reg);
    }
    return ret_code;
}

static int bq_core1_read_chg_registers(struct core1_battery_status *bat_out, struct core1_operation_status *opr_out) {
    // TODO fill this ups
}

static bool bq_core1_check_dsg_fet(void) {
    uint32_t op_stat;
    sbs_read_h4(&op_stat, BQ_READ_OPER_STAT);
    return (op_stat & 0x2) != 0;
}

static bool bq_core1_check_dsg(void) {
    uint16_t bat_stat;
    sbs_read_u2(&bat_stat, BQ_READ_PACK_STAT);
    return (bat_stat & 0x40) != 0;
}

/**
 * @brief read manufacturer info (name, date, serial number) and pass them to output varibles
 *
 * @param pack_info_out manufacturer info output variable
 * @return int return code of i2c transfer, 0 as successful, otherwise failed
 */
static int bq_core1_get_mfg_info(struct mfg_info *pack_info_out) {
    int ret_code = 0;
    uint8_t data[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // SERIAL
    uint8_t bq_reg[1] = { BQ_READ_CELL_SERI };
    ret_code |= bq_handle_i2c_transfer(bq_reg, data, 2);
    pack_info_out->serial = data[1] << 8 | data[0];

    // DATE
    bq_reg[0] = BQ_READ_CELL_DATE;
    ret_code |= bq_handle_i2c_transfer(bq_reg, data, 2);
    uint16_t raw_date = data[1] << 8 | data[0];
    pack_info_out->mfg_day = raw_date & 0x1f;
    pack_info_out->mfg_mo = (raw_date >> 5) & 0xf;
    pack_info_out->mfg_year = (raw_date >> 9) + 1980;

    // NAME
    bq_reg[0] = BQ_READ_CELL_NAME;
    ret_code |= bq_handle_i2c_transfer(bq_reg, data, 21);
    for (int i = 1; i < data[0]; i++) {
        pack_info_out->name[i - 1] = data[i];
    }
    pack_info_out->name[data[0] + 1] = 0;

    return ret_code;
}

/**
 * @brief flush data to shared_mfg_info via locking spin lock
 *
 * @param pack_info
 */
static void core1_flush_mfg_info(struct mfg_info *pack_info) {
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
 * @param opr_stat
 */
static void core1_flush_battery_info(struct core1_battery_status *bat_stat, struct core1_operation_status *opr_stat) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    shared_status.pres_flag = opr_stat->present;
    shared_status.safety_status_flag = opr_stat->safety_status;
    shared_status.status.voltage = bat_stat->voltage;
    shared_status.status.current = bat_stat->current;
    shared_status.status.avg_current = bat_stat->avg_current;
    shared_status.status.time_to_empty = bat_stat->time_to_empty;
    shared_status.status.soc = bat_stat->soc;
    shared_status.status.port_detected = bat_stat->port_detected;
    spin_unlock(shared_status.lock, irq);
}

void core1_update_soc_leds(uint8_t soc) {
    uint8_t state = 0;
    if (soc > WARN_SOC_THRESH) {
        state = 2;
    }
    else if (soc > STOP_SOC_THRESH) {
        state = 1;
    }
    for (uint8_t led = 0; led < 3; led++) {
        gpio_put(BQ_LEDS_CORE1[led], led == state);
    }
}

// ===========================================
// Entry point and control loop of core1
// ===========================================
static void __time_critical_func(core1_main)(void) {
    bq_core1_init();

    struct core1_battery_status local_bat_stat;
    struct core1_operation_status local_opr_stat;
    struct mfg_info local_mfg_info;

    while (1) {
        // ===================================================
        // Disconnected state
        // ===================================================
        if (!connected) {
            // Try connect via checking Serial Number
            // return mfg_pack_info for initialization
            if (try_connect()) {
                connected = true;
                /**
                 * @brief Only get mfg_info once. Read_once_cached only set to true
                 * when mfg_info is completely flushed to shared memory
                 *
                 */
                if (!read_once_cached) {
                    // TODO get device chemistry, cycle count, state of health, full charge capacity
                    // Check for i2c transaction error
                    int ret_code;
                    uint8_t retries = 0;
                    while (retries < RETRANSMIT_COUNT && (ret_code = bq_core1_get_mfg_info(&local_mfg_info))) {
                        retries++;
                    }
                    if (retries < RETRANSMIT_COUNT) {
                        core1_flush_mfg_info(&local_mfg_info);
                        read_once_cached = true;
                    }
                    else {
                        // failed to i2c transfer for RETRANSMIT_COUNT time
                        connected = false;
                        bq_error(BQ_ERROR_I2C_TRANSACTION, ret_code);
                    }
                }
            }
            else {
                bq_error(BQ_ERROR_START_UP_TOO_LONG, 0);
            }
        }
        // ===================================================
        // Connected state
        // ===================================================
        else if (connected) {
            // Check for i2c transaction error
            int ret_code;
            uint8_t retries = 0;
            while (retries < RETRANSMIT_COUNT &&
                   (ret_code = bq_core1_read_dsg_registers(&local_bat_stat, &local_opr_stat))) {
                retries++;
            }
            if (retries < RETRANSMIT_COUNT) {
                // i2c transfer succeed
                core1_flush_battery_info(&local_bat_stat, &local_opr_stat);
                core1_update_soc_leds(local_bat_stat.soc);
                // ====================================================
                // Handle multicore_fifo with any queued commands
                // ====================================================
                while (multicore_fifo_rvalid()) {
                    sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };

                    //  Power Cycle the battery
                    if (req.type == BQ_MAC_EMG_FET_CTRL_CMD) {
                        if (bq_core1_check_dsg_fet()) {
                            //  Send emergency manual FET control command
                            send_mac_command((uint16_t) req.cmd);

                            //  Send emergency FET OFF command
                            send_mac_command(0x043D);

                            absolute_time_t fet_off_time_end = make_timeout_time_ms(fet_open_time_ms);
                            while (bq_core1_check_dsg_fet() && !time_reached(fet_off_time_end)) {
                                sleep_us(100);
                                watchdog_update();
                            }
                            // Error checking
                            uint16_t status;
                            sbs_read_u2(&status, BQ_READ_PACK_STAT);
                            if (status & 0x7) {
                                char err[9] = "ERROR:#|#";
                                itoa(status & 0x7, err + 6, 10);
                                *(err + 7) = '|';
                                itoa((status & 0x700) >> 8, err + 8, 10);  // FIXME not sure what this is for
                                // FIXME just raise faults instead of panic
                                panic(err);
                            }
                            if (bq_core1_check_dsg_fet()) {
                                // bad state: FET is ON even after requesting FET OFF command
                                // Send device RESET command
                                send_mac_command(0x0041);
                                // FIXME just raise faults instead of panic
                                panic("Failed to open DSCHG FET during power cycle cmd");
                            }
                            //  Send emergency FET ON command
                            send_mac_command(0x23A7);
                        }
                    }
                }
            }
            else {
                // failed to i2c transfer for RETRANSMIT_COUNT time
                connected = false;
                bq_error(BQ_ERROR_I2C_TRANSACTION, ret_code);
            }
        }
    }
}

// ============================================
// Exported Function to core0
// ============================================

void core1_init(uint8_t expected_serial) {
    shared_status.lock = spin_lock_init(spin_lock_claim_unused(true));
    shared_mfg_info.lock = spin_lock_init(spin_lock_claim_unused(true));

    sbh_mcu_serial = expected_serial;
    multicore_launch_core1(core1_main);
}
bool core1_get_pack_mfg_info(struct mfg_info *pack_info_out) {
    bool read_successful = false;
    if (read_once_cached) {
        uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
        pack_info_out->serial = shared_mfg_info.pack_info.serial;
        pack_info_out->mfg_day = shared_mfg_info.pack_info.mfg_day;
        pack_info_out->mfg_mo = shared_mfg_info.pack_info.mfg_mo;
        pack_info_out->mfg_year = shared_mfg_info.pack_info.mfg_year;
        // TODO double check passing on array
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

void core1_get_battery_status(struct core1_battery_status *status_out) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    status_out->voltage = shared_status.status.voltage;
    status_out->current = shared_status.status.current;
    status_out->avg_current = shared_status.status.avg_current;
    status_out->time_to_empty = shared_status.status.time_to_empty;
    status_out->soc = shared_status.status.soc;
    status_out->port_detected = shared_status.status.port_detected;
    spin_unlock(shared_status.lock, irq);
}

bool core1_check_present(void) {
    bool present_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    present_out = shared_status.pres_flag;
    spin_unlock(shared_status.lock, irq);
    return present_out;
}

void core1_open_dsg_temp(const uint32_t open_time_ms) {
    fet_open_time_ms = open_time_ms;
    sio_fifo_req_t req = { .type = BQ_MAC_EMG_FET_CTRL_CMD, .cmd = 0x270C };
    multicore_fifo_push_blocking(req.raw);
}
