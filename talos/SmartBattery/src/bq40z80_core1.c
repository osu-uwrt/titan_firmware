#include "bq40z80_core1.h"

#include "pio_i2c.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "titan/logger.h"

#include <stdio.h>

#define RETRANSMIT_COUNT 4

uint8_t BQ_LEDS[3] = { LED_R_PIN, LED_Y_PIN, LED_G_PIN };
uint pio_12c_program;
uint32_t fet_open_time_ms;

#define MAX_TIME_STATUS_EXPIRED_MS 2000

// Width can't be below 15, since command takes up 16 bit
#define FIFO_REQ_VALUE_WIDTH 24  // TODO change if using more command

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
static volatile struct core1_status_shared_mem {
    spin_lock_t *lock;

    struct core1_battery_status status;

    bool safety_status_flag;

    uint8_t safety_status_reg[4];
} shared_status = { 0 };

static volatile struct core1_mfg_info_shared_mem {
    spin_lock_t *lock;

    struct mfg_info pack_info;

    bool read_once;
} shared_mfg_info = { 0 };

// FNC prototypes
static uint8_t bq_handle_i2c_transfer(uint8_t *bq_reg, uint8_t *rx_buf, uint len);
static uint8_t bq_write_only_transfer(uint8_t *tx_buf, uint len);
static void send_mac_command(uint16_t command);
static void bq_core1_init();
static struct mfg_info bq_core1_get_mfg_info();
static uint16_t bq_core1_get_voltage(void);
static int16_t bq_core1_get_current(void);
static int16_t bq_core1_get_avg_current(void);
static uint16_t bq_core1_get_time_empty(void);
static uint8_t bq_core1_get_rel_soc(void);
static struct core1_operation_status bq_core1_get_operation_status(void);
static void bq_core1_get_safety_status(uint8_t *data, uint8_t len);

static void __time_critical_func(core1_main)(void) {
    bq_core1_init();  // TODO add i2c check if i2c works for bq40z80

    // read serial, name, date locally
    struct mfg_info local_pack_info = bq_core1_get_mfg_info();

    while (1) {
        // ============================================
        // Pack Info read once
        // ============================================
        uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
        if (!shared_mfg_info.read_once) {
            shared_mfg_info.read_once = true;
            shared_mfg_info.pack_info.serial = local_pack_info.serial;
            shared_mfg_info.pack_info.mfg_day = local_pack_info.mfg_day;
            shared_mfg_info.pack_info.mfg_mo = local_pack_info.mfg_mo;
            shared_mfg_info.pack_info.mfg_year = local_pack_info.mfg_year;

            for (uint8_t i = 0; i < 21; i++) {
                shared_mfg_info.pack_info.name[i] = local_pack_info.name[i];
                if (shared_mfg_info.pack_info.name[i] == 0)
                    break;
            }
        }
        spin_unlock(shared_mfg_info.lock, irq);

        // ============================================
        // Continuous Reading(Volt,Curr,AvgCurr,TimeEmpty,SOC,PRES)
        // ============================================
        struct core1_operation_status local_op_status;
        local_op_status = bq_core1_get_operation_status();

        struct core1_battery_status local_battery_status;
        local_battery_status.voltage = bq_core1_get_voltage();
        local_battery_status.current = bq_core1_get_current();
        local_battery_status.avg_current = bq_core1_get_avg_current();
        local_battery_status.time_to_empty = bq_core1_get_time_empty();
        local_battery_status.soc = bq_core1_get_rel_soc();
        local_battery_status.present = local_op_status.present;

        irq = spin_lock_blocking(shared_status.lock);

        shared_status.status.voltage = local_battery_status.voltage;
        shared_status.status.current = local_battery_status.current;
        shared_status.status.avg_current = local_battery_status.avg_current;
        shared_status.status.time_to_empty = local_battery_status.time_to_empty;
        shared_status.status.soc = local_battery_status.soc;
        shared_status.status.present = local_battery_status.present;
        shared_status.safety_status_flag = local_op_status.safety_status;

        spin_unlock(shared_status.lock, irq);

        // ===================================================
        // Report any raised safety status bit
        // ===================================================
        if (local_op_status.safety_status) {
            uint8_t data[4] = { 0, 0, 0, 0 };
            bq_core1_get_safety_status(data, 4);
            irq = spin_lock_blocking(shared_status.lock);
            for (uint8_t i = 0; i < 4; i++) {
                shared_status.safety_status_reg[i] = data[i];
            }
            spin_unlock(shared_status.lock, irq);
        }

        // ===================================================
        // Handling multicore_fifo with any queued up commands
        // ===================================================

        while (multicore_fifo_rvalid()) {
            sio_fifo_req_t req = { .raw = multicore_fifo_pop_blocking() };
            if (req.type == BQ_MAC_EMG_FET_CTRL_CMD) {
                if (!local_op_status.discharge) {
                    multicore_fifo_drain();
                    break;
                }

                send_mac_command((uint16_t) req.cmd);
            }
            else if (req.type == BQ_MAC_EMG_FET_OFF_CMD) {
                send_mac_command((uint16_t) req.cmd);

                absolute_time_t fet_off_time_end = make_timeout_time_ms(fet_open_time_ms);
                while (local_op_status.discharge && !time_reached(fet_off_time_end)) {
                    sleep_us(100);
                    watchdog_update();
                }
                // TODO double check on this part
                uint8_t data[2] = { 0, 0 };
                uint8_t bq_reg[1] = { BQ_READ_PACK_STAT };
                bq_handle_i2c_transfer(bq_reg, data, 2);
                if (data[0] & 0x7) {
                    char err[9] = "ERROR:#|#";
                    itoa(data[0] & 0x7, err + 6, 10);
                    *(err + 7) = '|';
                    itoa(data[1] & 0x7, err + 8, 10);
                    panic(err);
                }
                if (local_op_status.discharge) {
                    send_mac_command(0x0041);
                    panic("Failed to open DSCHG FET during power cycle cmd");
                }
            }
            else if (req.type == BQ_MAC_EMG_FET_ON_CMD) {
                send_mac_command((uint16_t) req.cmd);
            }
        }
    }
}

// ============================================
// I2C Functions
// ============================================

static uint8_t bq_handle_i2c_transfer(uint8_t *bq_reg, uint8_t *rx_buf, uint len) {
    int ret_code = -1;  // if you set this to zero, the compiler *will delete* this function
    uint retries = 0;

    // slow down the transfers, smbus and pio no likey :/
    sleep_ms(1);

    while (ret_code && retries < RETRANSMIT_COUNT) {
        // this is a bug in the SM
        i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

        // send the request to the chip
        ret_code = 0;
        ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, bq_reg, 1);
        ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, rx_buf, len);

        if (ret_code) {
            // let i2c relax a sec, something with smbus and the chip being busy
            sleep_ms(3);
            retries++;
        }
    }

    if (ret_code) {
        // something bad happened to the i2c, panic
        LOG_FATAL("I2C transfer error: %d", ret_code);
        panic("PIO I2C NACK during transfer %d times", RETRANSMIT_COUNT);
    }

    return retries;
}

static uint8_t bq_write_only_transfer(uint8_t *tx_buf, uint len) {
    int ret_code = -1;  // if you set this to zero, the compiler *will delete* this function
    uint retries = 0;

    // slow down the transfers, smbus and pio no likey :/
    sleep_ms(1);

    while (ret_code && retries < RETRANSMIT_COUNT) {
        // this is a bug in the SM
        i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

        // send the request to the chip
        ret_code = pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, tx_buf, len);

        if (ret_code) {
            // let i2c relax a sec, something with smbus and the chip being busy
            sleep_ms(3);
            retries++;
        }
    }

    if (ret_code) {
        // something bad happened to the i2c, panic
        LOG_FATAL("I2C write error: %d", ret_code);
        panic("PIO I2C NACK during MAC_WRITE %d times", RETRANSMIT_COUNT);
    }

    return retries;
}

static void send_mac_command(uint16_t command) {
    uint8_t data[3] = {
        0x00,                        // MAC register address (actually 1 byte not 2 so cut off the upper byte)
        (uint8_t) (command >> 8),    // the high byte of the command
        (uint8_t) (command & 0xFF),  // the low byte of the command
    };
    bq_write_only_transfer(data, 3);
}

// ============================================
// BQ40Z80 Functions
// ============================================

static void bq_core1_init() {
    // Init shared_mfg_info flag
    uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
    shared_mfg_info.read_once = false;
    spin_unlock(shared_mfg_info.lock, irq);

    // Init the wake pin, active high
    gpio_init(BMS_WAKE_PIN);
    gpio_set_dir(BMS_WAKE_PIN, GPIO_OUT);
    gpio_put(BMS_WAKE_PIN, 0);

    for (uint8_t led = 0; led < 3; led++) {
        gpio_init(BQ_LEDS[led]);
        gpio_set_dir(BQ_LEDS[led], GPIO_OUT);
        gpio_put(BQ_LEDS[led], 0);
    }

    // init PIO I2C
    pio_12c_program = pio_add_program(pio0, &i2c_program);
    pio_sm_claim(pio0, PIO_SM);
    i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);
}

static struct mfg_info bq_core1_get_mfg_info() {
    struct mfg_info pack_info;
    uint8_t data[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // SERIAL
    uint8_t bq_reg[1] = { BQ_READ_CELL_SERI };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    pack_info.serial = data[1] << 8 | data[0];

    // DATE
    bq_reg[0] = BQ_READ_CELL_DATE;
    bq_handle_i2c_transfer(bq_reg, data, 2);
    uint16_t raw_date = data[1] << 8 | data[0];
    pack_info.mfg_day = raw_date & 0x1f;
    pack_info.mfg_mo = (raw_date >> 5) & 0xf;
    pack_info.mfg_year = (raw_date >> 9) + 1980;

    // NAME
    bq_reg[0] = BQ_READ_CELL_NAME;
    bq_handle_i2c_transfer(bq_reg, data, 21);
    for (int i = 1; i < data[0]; i++) {
        pack_info.name[i - 1] = data[i];
    }
    pack_info.name[data[0] + 1] = 0;

    return pack_info;
}

static uint16_t bq_core1_get_voltage(void) {
    uint8_t data[2] = { 0, 0 };
    uint8_t bq_reg[1] = { BQ_READ_PACK_VOLT };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    return data[1] << 8 | data[0];
}

static int16_t bq_core1_get_current(void) {
    uint8_t data[2] = { 0, 0 };
    uint8_t bq_reg[1] = { BQ_READ_PACK_CURR };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    int16_t curr_ma = data[1] << 8 | data[0];
    return curr_ma * 4;
}

static int16_t bq_core1_get_avg_current(void) {
    uint8_t data[2] = { 0, 0 };
    uint8_t bq_reg[1] = { BQ_READ_AVRG_CURR };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    int16_t curr_ma = data[1] << 8 | data[0];
    return curr_ma * 4;
}

static uint16_t bq_core1_get_time_empty(void) {
    uint8_t data[2] = { 0, 0 };
    uint8_t bq_reg[1] = { BQ_READ_TIME_EMPT };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    return data[1] << 8 | data[0];
}

static uint8_t bq_core1_get_rel_soc(void) {
    uint8_t data[1] = { 0 };
    uint8_t bq_reg[1] = { BQ_READ_RELAT_SOC };
    bq_handle_i2c_transfer(bq_reg, data, 1);
    return data[0];
}

static struct core1_operation_status bq_core1_get_operation_status(void) {
    struct core1_operation_status status_out;
    uint8_t data[5] = { 0, 0, 0, 0, 0 };
    uint8_t bq_reg[1] = { BQ_READ_OPER_STAT };
    bq_handle_i2c_transfer(bq_reg, data, 5);
    // the 0th byte is the length of data
    status_out.present = (data[1] & 0x1) == 1;
    status_out.discharge = ((data[1] & 0x2) >> 1) == 1;
    status_out.safety_status = ((data[2] & 0x8) >> 3) == 1;
    return status_out;
}

static void bq_core1_get_safety_status(uint8_t *data, uint8_t len) {
    uint8_t bq_reg[1] = { BQ_READ_SAFE_STAT };
    bq_handle_i2c_transfer(bq_reg, data, len);
}

// ============================================
// Exported Function to core0
// ============================================

void core1_init(void) {
    shared_status.lock = spin_lock_init(spin_lock_claim_unused(true));
    shared_mfg_info.lock = spin_lock_init(spin_lock_claim_unused(true));

    multicore_launch_core1(core1_main);
}
bool core1_get_pack_mfg_info(struct mfg_info *pack_info_out) {
    bool read_successful = false;
    uint32_t irq = spin_lock_blocking(shared_mfg_info.lock);
    if (shared_mfg_info.read_once) {
        read_successful = true;
        pack_info_out->serial = shared_mfg_info.pack_info.serial;
        pack_info_out->mfg_day = shared_mfg_info.pack_info.mfg_day;
        pack_info_out->mfg_mo = shared_mfg_info.pack_info.mfg_mo;
        pack_info_out->mfg_year = shared_mfg_info.pack_info.mfg_year;
        // TODO double check on passing on array
        for (uint8_t i = 0; i < 21; i++) {
            pack_info_out->name[i] = shared_mfg_info.pack_info.name[i];
            if (pack_info_out->name[i] == 0)
                break;
        }
    }

    spin_unlock(shared_mfg_info.lock, irq);
    return read_successful;
}

void core1_get_shared_status(struct core1_battery_status *status_out) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    status_out->voltage = shared_status.status.voltage;
    status_out->current = shared_status.status.current;
    status_out->avg_current = shared_status.status.avg_current;
    status_out->time_to_empty = shared_status.status.time_to_empty;
    status_out->soc = shared_status.status.soc;
    status_out->present = shared_status.status.present;
    spin_unlock(shared_status.lock, irq);
}

bool core1_check_safety_status(void) {
    bool status_out;
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    status_out = shared_status.safety_status_flag;
    spin_unlock(shared_status.lock, irq);
    return status_out;
}

void core1_open_dsg_temp(const uint32_t open_time_ms) {
    fet_open_time_ms = open_time_ms;
    sio_fifo_req_t req = { .type = BQ_MAC_EMG_FET_CTRL_CMD, .cmd = 0x270C };
    multicore_fifo_push_blocking(req.raw);
    req.type = BQ_MAC_EMG_FET_OFF_CMD;
    req.cmd = 0x043D;
    multicore_fifo_push_blocking(req.raw);
    req.type = BQ_MAC_EMG_FET_ON_CMD;
    req.cmd = 0x23A7;
    multicore_fifo_push_blocking(req.raw);
}

void core1_get_safety_status_reg(uint8_t *data, uint8_t len) {
    uint32_t irq = spin_lock_blocking(shared_status.lock);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = shared_status.safety_status_reg[i];
    }
    spin_unlock(shared_status.lock, irq);
}
