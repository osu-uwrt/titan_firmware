#define RUNNING_ON_CORE1
#include "bq40z80.h"

#include "pio_smbus.h"
#include "safety_interface.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include <stdio.h>
#include <string.h>

uint8_t BQ_LEDS[3] = { LED_R_PIN, LED_Y_PIN, LED_G_PIN };
uint pio_i2c_program;

static bool battery_connected;
static int err_count = 2;

// ========================================
// BQ40 SMBus Interface
// All direct calls to pio_smbus must live here!
// ========================================

// Throttles bq40 smbus transfers since if you send faster than 150 us it'll sometimes NAK
#define BQ_THROTTLE_TIME_US 150
static absolute_time_t bq_transfer_throttle;

/**
 * @brief Performs an SMBus word read for the specific command code on the bq40
 *
 * @param cmd The SMBus command to perform the word read on
 * @param word_out Pointer to write the received word to
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error
 */
static int bq_read_word(uint8_t cmd, uint16_t *word_out) {
    // Throttle transfers if needed
    if (!time_reached(bq_transfer_throttle)) {
        sleep_until(bq_transfer_throttle);
    }

    int ret = pio_smbus_word_read_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, cmd);
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
    if (ret < 0) {
        return ret;
    }
    else {
        *word_out = (uint16_t) ret;
        return PIO_SMBUS_SUCCESS;
    }
}

/**
 * @brief Performs an SMBus block read for the specific command code on the bq40
 *
 * @param cmd The SMBus command to perform the block read on
 * @param rxbuf Buffer to store response from device
 * @param len Pointer containing the capacity of rxbuf. This will be updated to the number of bytes written to rxbuf
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error. On error, len is not updated
 */
static int bq_read_block(uint8_t cmd, uint8_t *rxbuf, size_t *len) {
    // Throttle if needed
    if (!time_reached(bq_transfer_throttle)) {
        sleep_until(bq_transfer_throttle);
    }

    // Do the transfer
    int ret = pio_smbus_block_read_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, cmd, rxbuf, *len);
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);

    // Handle result
    if (ret < 0) {
        return ret;
    }
    else {
        *len = ret;
        return PIO_SMBUS_SUCCESS;
    }
}

/**
 * @brief Reads the requested 16-bit Manufacturer Access Block
 * This will perform an SMBus read, up to max_len bytes
 * If max_len is smaller than the reported number of bytes, an error will occur
 *
 * @param mac_cmd The Manufacturer Access Command to execute
 * @param rxbuf Buffer to store response from device
 * @param len Pointer containing the capacity of rxbuf. This will be updated to the number of bytes written to rxbuf
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error. On error, len is not updated
 */
static int bq_mfg_access_read(uint16_t mac_cmd, uint8_t *rxbuf, size_t *len) {
    if (*len > UINT8_MAX - 2) {
        return PIO_SMBUS_ERR_BUF_TOO_LARGE;
    }

    // Throttle if needed
    if (!time_reached(bq_transfer_throttle)) {
        sleep_until(bq_transfer_throttle);
    }

    // Issue the MAC cmd
    uint8_t mac_cmd_buf[] = { mac_cmd & 0xFF, mac_cmd >> 8 };
    int ret =
        pio_smbus_block_write_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, BQ_MFG_BLK_ACCESS, mac_cmd_buf, sizeof(mac_cmd_buf));
    if (ret != PIO_SMBUS_SUCCESS) {
        bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
        return ret;
    }

    // Receive response
    // No need to throttle here it seems
    uint8_t mac_resp[*len + 2];
    ret = pio_smbus_block_read_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, BQ_MFG_BLK_ACCESS, mac_resp, sizeof(mac_resp));
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
    if (ret < 0)
        return ret;

    // The command must at least contain the command we just sent
    if (ret < 2 || rxbuf[0] != mac_cmd_buf[0] || rxbuf[1] != mac_cmd_buf[1]) {
        return PIO_SMBUS_ERR_INVALID_RESP;
    }

    // Now copy the data that the caller cares about out
    size_t resp_len = ret - 2;
    memcpy(rxbuf, &mac_resp[2], resp_len);
    *len = resp_len;
    return PIO_SMBUS_SUCCESS;
}

/**
 * @brief Sends the requested 16-bit Manufacturer Access Command
 * This only send the command, with no data, and does not receive a response
 *
 * @param mac_cmd The Manufacturer Access Command to execute
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error
 */
static int bq_mfg_access_cmd(uint16_t mac_cmd) {
    // Throttle if needed
    if (!time_reached(bq_transfer_throttle)) {
        sleep_until(bq_transfer_throttle);
    }

    // Issue the MAC cmd
    uint8_t mac_cmd_buf[] = { mac_cmd & 0xFF, mac_cmd >> 8 };
    int ret =
        pio_smbus_block_write_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, BQ_MFG_BLK_ACCESS, mac_cmd_buf, sizeof(mac_cmd_buf));
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
    return ret;
}

static int bq_read_dword(uint8_t cmd, uint32_t *dword_out) {
    uint32_t word;
    size_t len = sizeof(word);
    int ret = bq_read_block(cmd, (uint8_t *) (&word), &len);
    if (ret) {
        return ret;
    }
    else if (len != sizeof(word)) {
        return PIO_SMBUS_ERR_INVALID_RESP;
    }
    else {
        *dword_out = word;
        return PIO_SMBUS_SUCCESS;
    }
}

// ========================================
// BQ40 Convenience I/O Wrapper
// ========================================

static int bq_read_sword(uint8_t cmd, int16_t *sword_out) {
    return bq_read_word(cmd, (uint16_t *) sword_out);
}

/**
 * @brief Performs an SMBus word read for the specific command code on the bq40, but enforces only 8 bits
 *
 * @param cmd The SMBus command to perform the word read on
 * @param byte_out Pointer to write the received byte to
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error
 */
static int bq_read_byte(uint8_t cmd, uint8_t *byte_out) {
    uint16_t word;
    int ret = bq_read_word(cmd, &word);
    if (ret) {
        return ret;
    }
    if (word > UINT8_MAX) {
        return PIO_SMBUS_ERR_INVALID_RESP;
    }
    else {
        *byte_out = (uint8_t) word;
        return PIO_SMBUS_SUCCESS;
    }
}

// Add checks into all code, and make sure this check is valid
#define I2CCHECK(func)                                                                                                 \
    do {                                                                                                               \
        int ret = func;                                                                                                \
        if (ret)                                                                                                       \
            return false;                                                                                              \
    } while (0)

// ========================================
// BQ40 Convenience Wrappers
// ========================================

/**
 * @brief read bq40z80 manufactured date and name
 *
 * @param mfg_out
 */
static bool bq40z80_read_mfg_info(bq_mfg_info_t *mfg_out) {
    // Date
    uint16_t raw_date;
    I2CCHECK(bq_read_word(BQ_READ_CELL_DATE, &raw_date));
    mfg_out->mfg_day = raw_date & 0x1f;
    mfg_out->mfg_mo = (raw_date >> 5) & 0xf;
    mfg_out->mfg_year = (raw_date >> 9) + 1980;

    // Name
    size_t len = sizeof(mfg_out->name) - 1;
    I2CCHECK(bq_read_block(BQ_READ_CELL_NAME, (uint8_t *) mfg_out->name, &len));
    mfg_out->name[len] = 0;

    return true;
}

/**
 * @brief read bq40z80 SOC, average current, DSG/CHG mode and their respective voltage, current, and time
 *
 * @param bat_out
 */
static bool bq40z80_read_battery_info(bq_battery_info_t *bat_out) {
    uint16_t data;
    uint32_t safety_status;

    I2CCHECK(bq_read_word(BQ_READ_GPIO, &data));
    bat_out->port_detected = (data & 0x8) != 0;
    I2CCHECK(bq_read_word(BQ_READ_PACK_STAT, &data));
    bat_out->dsg_mode = (data & 0x40) ? true : false;
    I2CCHECK(bq_read_word(BQ_READ_PACK_VOLT, &bat_out->voltage));
    // TODO: Fix current scaling!!
    I2CCHECK(bq_read_sword(BQ_READ_PACK_CURR, &bat_out->current));
    I2CCHECK(bq_read_word(BQ_READ_TIME_EMPT, &bat_out->time_to_empty));
    I2CCHECK(bq_read_word(BQ_READ_CHG_VOLT, &bat_out->chg_voltage));
    I2CCHECK(bq_read_word(BQ_READ_CHG_CURR, &bat_out->chg_current));
    I2CCHECK(bq_read_word(BQ_READ_TIME_FULL, &bat_out->time_to_full));
    I2CCHECK(bq_read_sword(BQ_READ_AVRG_CURR, &bat_out->avg_current));
    I2CCHECK(bq_read_byte(BQ_READ_RELAT_SOC, &bat_out->soc));
    // Checks for Battery Presense and Safety Status alarms
    I2CCHECK(bq_read_dword(BQ_READ_OPER_STAT, &safety_status));
    bat_out->battery_presence = (safety_status & 0x1) != 0;
    if ((safety_status & 0x800) != 0) {
        I2CCHECK(bq_read_dword(BQ_READ_SAFE_STAT, &safety_status));
        safety_raise_fault_with_arg(FAULT_BQ40_SAFETY_STATUS, safety_status);
    }
    return true;
}

void bq40z80_update_soc_leds(uint8_t soc) {
    uint8_t state = 0;
    if (soc > WARN_SOC_THRESH) {
        state = 2;
    }
    else if (soc > STOP_SOC_THRESH) {
        state = 1;
    }
    for (uint8_t led = 0; led < 3; led++) {
        gpio_put(BQ_LEDS[led], led == state);
    }
}

void bq40z80_init() {
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
    pio_i2c_program = pio_add_program(BQ_PIO_INST, &i2c_program);
    pio_sm_claim(BQ_PIO_INST, BQ_PIO_SM);
    i2c_program_init(BQ_PIO_INST, BQ_PIO_SM, pio_i2c_program, BMS_SDA_PIN, BMS_SCL_PIN);

    battery_connected = false;
}

bool bq40z80_refresh_reg(uint8_t sbh_mcu_serial, bool read_once, bq_battery_info_t *bat_out, bq_mfg_info_t *mfg_out) {
    uint16_t serial;
    // Better handling between serial errors and connection faults
    int ret = bq_read_word(BQ_READ_CELL_SERI, &serial);
    if (ret || serial == 0xffff || serial == 0x0000) {
        // handle refresh fail error
        if (battery_connected) {
            err_count++;
            if (err_count >= 3) {
                battery_connected = false;
                safety_raise_fault_with_arg(FAULT_BQ40_NOT_CONNECTED, -ret);
            }
        }
        else {
            safety_raise_fault_with_arg(FAULT_BQ40_NOT_CONNECTED, -ret);
            // wake up battery and wait for 1 second
            gpio_put(BMS_WAKE_PIN, 1);
            sleep_ms(1000);
            gpio_put(BMS_WAKE_PIN, 0);
        }

        return false;
    }
    else if (serial != sbh_mcu_serial) {
        safety_raise_fault_with_arg(FAULT_BQ40_MISMATCHED_SERIAL, (sbh_mcu_serial << 16) | serial);
        // TODO: Should this be fatal?
        return false;
    }
    else {
        err_count = 0;
        if (!battery_connected) {
            battery_connected = true;
        }
        if (!read_once) {
            // TODO: read_once should probably be related to connection sequence
            mfg_out->serial = serial;
            bq40z80_read_mfg_info(mfg_out);
        }
        bq40z80_read_battery_info(bat_out);
        return true;
    }
}

static bool bq_pack_discharging() {
    // TODO: Better error handling
    uint32_t oper_stat;
    I2CCHECK(bq_read_dword(BQ_READ_OPER_STAT, &oper_stat));

    // test the discharge bit (bit 1)
    return !!(oper_stat & 0x2);
}

// WARNING! This is a blocking call. It should block for around 10s. It will continue to feed the
// watchdog during execution as to not time the system out. This must be done to maintain MAC synchronicity
// with the BQ chip during manual fet control
bool bq_open_dschg_temp(const int64_t open_time_ms) {
    // test operation status to determine if output is not on
    if (!bq_pack_discharging()) {
        // bail, dont want to tun on the output manually
        return false;
    }

    // send Emergency FET override command to take manual control
    I2CCHECK(bq_mfg_access_cmd(BQ_MAC_EMG_FET_CTRL_CMD));

    // send emergency fet off command
    I2CCHECK(bq_mfg_access_cmd(BQ_MAC_EMG_FET_OFF_CMD));

    absolute_time_t fet_off_command_end = make_timeout_time_ms(open_time_ms);
    while (bq_pack_discharging() && !time_reached(fet_off_command_end)) {
        sleep_ms(10);
        // also feed the watchdog so we dont reset
        safety_core1_checkin();
    }

    uint16_t pack_stat;
    I2CCHECK(bq_read_word(BQ_READ_PACK_STAT, &pack_stat));
    if (pack_stat & 0x7) {
        uint32_t err_code = (pack_stat & 0x7);
        safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, err_code);
    }

    // verify fet is actually open via operation status
    if (bq_pack_discharging()) {
        // this is a bad state. we requested fet open and it didnt happen
        I2CCHECK(bq_mfg_access_cmd(BQ_MAC_RESET_CMD));
        safety_raise_fault_with_arg(FAULT_BQ40_COMMAND_FAIL, 0);
    }

    // send a cleanup reset command
    I2CCHECK(bq_mfg_access_cmd(BQ_MAC_EMG_FET_ON_CMD));

    return true;
}

bool bq_cycle_count(uint16_t *cycles_out) {
    I2CCHECK(bq_read_word(BQ_READ_CYCLE_COUNT, cycles_out));
    return true;
}
