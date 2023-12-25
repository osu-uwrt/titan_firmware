#include "bq40z80.h"

#include "pio_i2c.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include <stdio.h>

uint8_t BQ_LEDS[3] = { LED_R_PIN, LED_Y_PIN, LED_G_PIN };
uint pio_i2c_program;
static bq40z80_error_cb client_error_callback;
static bool battery_connected;
static int err_count = 2;

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
 * @brief read bq40z80 manufactured date and name
 *
 * @param mfg_out
 */
static void bq40z80_read_mfg_info(bq_mfg_info_t *mfg_out) {
    uint8_t data[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // Date
    uint8_t bq_reg[1] = { BQ_READ_CELL_DATE };
    bq_handle_i2c_transfer(bq_reg, data, 2);
    uint16_t raw_date = data[1] << 8 | data[0];
    mfg_out->mfg_day = raw_date & 0x1f;
    mfg_out->mfg_mo = (raw_date >> 5) & 0xf;
    mfg_out->mfg_year = (raw_date >> 9) + 1980;

    // Name
    bq_reg[0] = BQ_READ_CELL_NAME;
    bq_handle_i2c_transfer(bq_reg, data, 21);
    for (int i = 1; i < data[0]; i++) {
        mfg_out->name[i - 1] = data[i];
    }
    mfg_out->name[data[0] + 1] = 0;
}

/**
 * @brief read bq40z80 SOC, average current, DSG/CHG mode and their respective voltage, current, and time
 *
 * @param bat_out
 */
static void bq40z80_read_battery_info(bq_battery_info_t *bat_out) {
    uint16_t data;
    uint32_t safety_status;

    sbs_read_u2(&data, BQ_READ_GPIO);
    bat_out->port_detected = (data & 0x8) != 0;
    sbs_read_u2(&data, BQ_READ_PACK_STAT);
    if (data & 0x40) {
        // DISCHARGING mode
        bat_out->dsg_mode = true;
        sbs_read_u2(&bat_out->voltage, BQ_READ_PACK_VOLT);
        sbs_read_i2(&bat_out->current, BQ_READ_PACK_CURR);
        sbs_read_u2(&bat_out->time_to_empty, BQ_READ_TIME_EMPT);
    }
    else {
        // CHARGING mode
        bat_out->dsg_mode = false;
        sbs_read_u2(&bat_out->chg_voltage, BQ_READ_CHG_VOLT);
        sbs_read_u2(&bat_out->chg_current, BQ_READ_CHG_CURR);
        sbs_read_u2(&bat_out->time_to_full, BQ_READ_TIME_FULL);
    }
    sbs_read_i2(&bat_out->avg_current, BQ_READ_AVRG_CURR);
    sbs_read_u1(&bat_out->soc, BQ_READ_RELAT_SOC);
    // Checks for Battery Presense and Safety Status alarms
    sbs_read_h4(&safety_status, BQ_READ_OPER_STAT);
    bat_out->battery_presence = (safety_status & 0x1) != 0;
    if ((safety_status & 0x800) != 0) {
        sbs_read_h4(&safety_status, BQ_READ_SAFE_STAT);
        client_error_callback(BQ_ERROR_SAFETY_STATUS, safety_status);
    }
}

static void bq40z80_update_soc_leds(uint8_t soc) {
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

static void bq40z80_on_error() {
    // handle refresh fail error
    if (battery_connected) {
        err_count++;
        if (err_count >= 3) {
            battery_connected = false;
            client_error_callback(BQ_ERROR_I2C_DISCONNECT, 0);
        }
    }
    else {
        client_error_callback(BQ_ERROR_OFFLINE, 0);
        // wake up battery and wait for 1 second
        gpio_put(BMS_WAKE_PIN, 1);
        sleep_ms(1000);
        gpio_put(BMS_WAKE_PIN, 0);
    }
}

/**
 * @brief initialize PIO hardware for i2c usage, GPIO for BQ_LEDS_CORE1, and BMS_WAKE_PIN
 *
 */
void bq40z80_init(bq40z80_error_cb error_cb) {
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
    pio_i2c_program = pio_add_program(pio0, &i2c_program);
    pio_sm_claim(pio0, PIO_SM);
    i2c_program_init(pio0, PIO_SM, pio_i2c_program, BMS_SDA_PIN, BMS_SCL_PIN);

    client_error_callback = error_cb;

    battery_connected = false;
}

/**
 * @brief first check serial number, and if matched it does other register reads
 *
 * @param connected the current state of battery
 * @param sbh_mcu_serial the expected serial num, used for checking if battery is connected
 * @param bat_out
 * @param mfg_out
 * @return true     i2c works and the battery is connected
 * @return false    i2c fails due to battery offline
 */
bool bq40z80_refresh_reg(uint8_t sbh_mcu_serial, bool read_once, bq_battery_info_t *bat_out, bq_mfg_info_t *mfg_out) {
    uint16_t serial;
    sbs_read_u2(&serial, BQ_READ_CELL_SERI);
    if (serial == sbh_mcu_serial) {
        err_count = 0;
        if (!battery_connected) {
            battery_connected = true;
        }
        if (!read_once) {
            mfg_out->serial = serial;
            bq40z80_read_mfg_info(mfg_out);
        }
        bq40z80_read_battery_info(bat_out);
        bq40z80_update_soc_leds(bat_out->soc);
        return true;
    }
    else {
        bq40z80_on_error();
        return false;
    }
}

static uint8_t bq_pack_discharging() {
    // read the operationstatus register (32 bits)
    uint8_t data[5] = { 0, 0, 0, 0, 0 };
    uint8_t reg_addr[1] = { BQ_READ_OPER_STAT };
    bq_handle_i2c_transfer(reg_addr, data, 5);

    // the zeroth byte is the length of the field for some reason...
    // test the discharge bit (bit 1)
    return (uint8_t) (data[1] & 0b00000010);
}

// WARNING! This is a blocking call. It should block for around 10s. It will continue to feed the
// watchdog during execution as to not time the system out. This must be done to maintain MAC synchronicity
// with the BQ chip during manual fet control
void bq_open_dschg_temp(const int64_t open_time_ms) {
    // test operation status to determine if output is not on
    if (!bq_pack_discharging()) {
        // bail, dont want to tun on the output manually
        return;
    }

    // send Emergency FET override command to take manual control
    send_mac_command(BQ_MAC_EMG_FET_CTRL_CMD);

    // send emergency fet off command
    send_mac_command(BQ_MAC_EMG_FET_OFF_CMD);

    absolute_time_t fet_off_command_end = make_timeout_time_ms(open_time_ms);
    while (bq_pack_discharging() && !time_reached(fet_off_command_end)) {
        sleep_us(100);
        // also feed the watchdog so we dont reset
        watchdog_update();
    }

    uint8_t data[2] = { 0, 0 };
    uint8_t reg_addr[1] = { BQ_READ_PACK_STAT };
    bq_handle_i2c_transfer(reg_addr, data, 2);
    if (data[0] & 0x7) {
        // char err[9] = "ERROR:#|#";
        // itoa(data[0] & 0x7, err + 6, 10);
        // *(err + 7) = '|';
        // itoa(data[1] & 0x7, err + 8, 10);
        // panic(err);
        uint32_t err_code = (data[0] & 0x7);
        client_error_callback(BQ_ERROR_POWER_CYCLE_FAIL, err_code);
    }

    // verify fet is actually open via operation status
    if (bq_pack_discharging()) {
        // this is a bad state. we requested fet open and it didnt happen
        send_mac_command(BQ_MAC_RESET_CMD);
        client_error_callback(BQ_ERROR_POWER_CYCLE_FAIL, 0);
    }

    // send a cleanup reset command
    send_mac_command(BQ_MAC_EMG_FET_ON_CMD);
}
