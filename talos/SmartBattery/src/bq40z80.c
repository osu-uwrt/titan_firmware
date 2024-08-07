#define RUNNING_ON_CORE1
#include "bq40z80.h"

#include "pio_smbus.h"

#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "bq40z80"

const uint8_t BQ_LEDS[3] = { LED_R_PIN, LED_Y_PIN, LED_G_PIN };

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
    int ret = pio_smbus_block_write_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, SBS_CMD_MANUFACTURER_BLOCK_ACCESS, mac_cmd_buf,
                                        sizeof(mac_cmd_buf));
    if (ret != PIO_SMBUS_SUCCESS) {
        bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
        return ret;
    }

    // Receive response
    // No need to throttle here it seems
    uint8_t mac_resp[*len + 2];
    ret = pio_smbus_block_read_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, SBS_CMD_MANUFACTURER_BLOCK_ACCESS, mac_resp,
                                   sizeof(mac_resp));
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
    if (ret < 0)
        return ret;

    // The command must at least contain the command we just sent
    if (ret < 2 || mac_resp[0] != mac_cmd_buf[0] || mac_resp[1] != mac_cmd_buf[1]) {
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
    int ret = pio_smbus_block_write_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, SBS_CMD_MANUFACTURER_BLOCK_ACCESS, mac_cmd_buf,
                                        sizeof(mac_cmd_buf));
    bq_transfer_throttle = make_timeout_time_us(BQ_THROTTLE_TIME_US);
    return ret;
}

/**
 * @brief Writes the requested manufacturer access block
 * This only sends the command, and data, not response is read
 *
 * @param mac_cmd The Manufacturer Access Command command to write
 * @param txbuf The buffer to write to the manufacturer access command
 * @param len The length of the buffer to write
 * @return int PIO_SMBUS_SUCCESS on success, or a negative error code on error
 */
static int bq_mfg_access_write(uint16_t mac_cmd, const uint8_t *txbuf, size_t len) {
    // Throttle if needed
    if (!time_reached(bq_transfer_throttle)) {
        sleep_until(bq_transfer_throttle);
    }

    if (len > 32 || len == 0) {
        return PIO_SMBUS_ERR_BUF_TOO_LARGE;
    }

    // Generate the MAC command
    uint8_t mac_cmd_buf[2 + len];
    mac_cmd_buf[0] = mac_cmd & 0xFF;
    mac_cmd_buf[1] = mac_cmd >> 8;
    memcpy(mac_cmd_buf + 2, txbuf, len);

    // Issue the MAC command
    int ret = pio_smbus_block_write_pec(BQ_PIO_INST, BQ_PIO_SM, BQ_ADDR, SBS_CMD_MANUFACTURER_BLOCK_ACCESS, mac_cmd_buf,
                                        sizeof(mac_cmd_buf));
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

static int bq_read_mfg_block_fixedlen(uint8_t cmd, void *rxbuf, size_t len) {
    size_t lenout = len;
    int ret = bq_mfg_access_read(cmd, (uint8_t *) rxbuf, &lenout);
    if (ret) {
        return ret;
    }
    if (len != lenout) {
        return PIO_SMBUS_ERR_INVALID_RESP;
    }
    else {
        return PIO_SMBUS_SUCCESS;
    }
}

// Add checks into all code, and make sure this check is valid
#define I2CCHECK(func)                                                                                                 \
    do {                                                                                                               \
        int rc = (func);                                                                                               \
        if (rc) {                                                                                                      \
            bq_error_t ret = { .fields = { .error_code = rc, .line = __LINE__ } };                                     \
            return ret;                                                                                                \
        }                                                                                                              \
    } while (0)

#define BQERRCHECK(func)                                                                                               \
    do {                                                                                                               \
        bq_error_t ret = (func);                                                                                       \
        if (!BQ_CHECK_SUCCESSFUL(ret))                                                                                 \
            return ret;                                                                                                \
    } while (0)

#define BQ_RETURN_SUCCESS                                                                                              \
    do {                                                                                                               \
        bq_error_t ret = { .fields = { .error_code = BQ_ERROR_SUCCESS, .line = 0, .arg = 0 } };                        \
        return ret;                                                                                                    \
    } while (0)

#define BQ_RETURN_ERROR(bq_error)                                                                                      \
    do {                                                                                                               \
        bq_error_t ret = { .fields = { .error_code = (bq_error), .line = __LINE__, .arg = 0 } };                       \
        return ret;                                                                                                    \
    } while (0)

#define BQ_RETURN_ERROR_WITH_ARG(bq_error, arg_val)                                                                    \
    do {                                                                                                               \
        bq_error_t ret = { .fields = { .error_code = (bq_error), .line = __LINE__, .arg = (uint8_t) (arg_val) } };     \
        return ret;                                                                                                    \
    } while (0)

static bq_error_t bq_check_status_successful(void) {
    uint16_t batt_status;
    I2CCHECK(bq_read_word(SBS_CMD_BATTERY_STATUS, &batt_status));
    uint8_t err_code = (batt_status & SBS_BATTERY_STATUS_EC_MASK);
    if (err_code) {
        BQ_RETURN_ERROR_WITH_ARG(BQ_ERROR_BATT_STATUS_ERROR, err_code);
    }

    BQ_RETURN_SUCCESS;
}

// ========================================
// BQ40 Data Flash Read Functions
// ========================================

typedef union bq_data_flash_int {
    uint8_t u1;
    uint16_t u2;
    uint32_t u4;
    int8_t i1;
    int16_t i2;
    int32_t i4;
} bq_data_flash_int_t;

static bq_error_t bq_read_data_flash_int(uint16_t flash_addr, bq_data_flash_int_t *data_out) {
    uint8_t read_buf[32];
    size_t len = sizeof(read_buf);
    I2CCHECK(bq_mfg_access_read(flash_addr, read_buf, &len));
    if (len < 4) {
        BQ_RETURN_ERROR_WITH_ARG(BQ_ERROR_INVALID_MAC_RESP_LEN, len);
    }
    data_out->u4 = read_buf[0] | ((uint32_t) (read_buf[1]) << 8) | ((uint32_t) (read_buf[2]) << 16) |
                   ((uint32_t) (read_buf[3]) << 24);
    BQ_RETURN_SUCCESS;
}

// ========================================
// BQ40 Refresh Routines
// ========================================

bq_error_t bq_read_mfg_info(bq_mfg_info_t *mfg_out) {
    // Device Type
    I2CCHECK(bq_read_mfg_block_fixedlen(MFG_CMD_DEVICE_TYPE, &mfg_out->device_type, sizeof(mfg_out->device_type)));
    if (mfg_out->device_type != 0x4800) {
        BQ_RETURN_ERROR_WITH_ARG(BQ_ERROR_INVALID_DEVICE_TYPE, mfg_out->device_type);
    }

    // Serial Number
    uint16_t serial;
    I2CCHECK(bq_read_word(SBS_CMD_SERIAL_NUMBER, &serial));
    if (serial == 0 || serial == 0xFFFF) {
        // These are invalid serial numbers, and probably mean I2C communication issues (or an unprogrammed chip)
        // Refuse to continue
        BQ_RETURN_ERROR_WITH_ARG(BQ_ERROR_INVALID_SERIAL, serial);
    }
    mfg_out->serial = serial;

    // Date
    uint16_t raw_date;
    I2CCHECK(bq_read_word(SBS_CMD_MANUFACTURER_DATE, &raw_date));
    mfg_out->mfg_day = raw_date & 0x1f;
    mfg_out->mfg_mo = (raw_date >> 5) & 0xf;
    mfg_out->mfg_year = (raw_date >> 9) + 1980;

    // Firmware Version
    I2CCHECK(bq_read_mfg_block_fixedlen(MFG_CMD_FIRMWARE_VERSION, mfg_out->firmware_version,
                                        sizeof(mfg_out->firmware_version)));

    // Current Scale Factor
    bq_data_flash_int_t scale_factor;
    BQERRCHECK(bq_read_data_flash_int(0x4AE8, &scale_factor));
    mfg_out->scale_factor = scale_factor.u1;

    // Manufacturing Status
    // TI is weird- made manufacturing a block read but only 2 bytes long
    I2CCHECK(bq_read_mfg_block_fixedlen(MFG_CMD_MANUFACTURING_STATUS, (uint8_t *) &mfg_out->manufacturing_status,
                                        sizeof(mfg_out->manufacturing_status)));

    BQ_RETURN_SUCCESS;
}

bq_error_t bq_read_battery_info(const bq_mfg_info_t *mfg_info, bq_battery_info_t *bat_out) {
    int16_t sdata;

    // Make sure the serial number matches the expected serial number
    // Final sanity check to make sure comms are working properly
    uint16_t serial;
    I2CCHECK(bq_read_word(SBS_CMD_SERIAL_NUMBER, &serial));
    if (serial != mfg_info->serial) {
        BQ_RETURN_ERROR_WITH_ARG(BQ_ERROR_SERIAL_CHANGED, serial);
    }

    // Refresh all of the fields we care about
    I2CCHECK(bq_read_dword(SBS_CMD_OPERATION_STATUS, &bat_out->operation_status));
    if (sbs_check_bit(bat_out->operation_status, SBS_OPERATION_STATUS_INIT)) {
        // Can't read anything else while initializing
        BQ_RETURN_SUCCESS;
    }

    I2CCHECK(bq_read_mfg_block_fixedlen(MFG_CMD_DA_STATUS1, &bat_out->da_status1, sizeof(bat_out->da_status1)));
    I2CCHECK(bq_read_sword(SBS_CMD_CURRENT, &sdata));
    bat_out->current = ((int32_t) sdata) * mfg_info->scale_factor;
    I2CCHECK(bq_read_sword(SBS_CMD_AVERAGE_CURRENT, &sdata));
    bat_out->avg_current = ((int32_t) sdata) * mfg_info->scale_factor;
    I2CCHECK(bq_read_word(SBS_CMD_TEMPERATURE, &bat_out->temperature));
    I2CCHECK(bq_read_byte(SBS_CMD_MAX_ERROR, &bat_out->max_error));
    I2CCHECK(bq_read_byte(SBS_CMD_RELATIVE_STATE_OF_CHARGE, &bat_out->relative_soc));
    I2CCHECK(bq_read_word(SBS_CMD_AVERAGE_TIME_TO_EMPTY, &bat_out->time_to_empty));
    I2CCHECK(bq_read_word(SBS_CMD_AVERAGE_TIME_TO_FULL, &bat_out->time_to_full));
    I2CCHECK(bq_read_word(SBS_CMD_BATTERY_STATUS, &bat_out->battery_status));
    I2CCHECK(bq_read_word(SBS_CMD_CELL_VOLTAGE_5, &bat_out->cell5_voltage));
    I2CCHECK(bq_read_word(SBS_CMD_CHARGING_VOLTAGE, &bat_out->charging_voltage));
    I2CCHECK(bq_read_sword(SBS_CMD_CHARGING_CURRENT, &sdata));
    bat_out->charging_current = ((int32_t) sdata) * mfg_info->scale_factor;

    // Read the safety status/pf status if needed (operationstatus has logical or of these fields)
    if (sbs_check_bit(bat_out->operation_status, SBS_OPERATION_STATUS_SS)) {
        I2CCHECK(bq_read_dword(SBS_CMD_SAFETY_STATUS, &bat_out->safety_status));
    }
    else {
        // Safety status is 0 since the logical or of the field is 0
        bat_out->safety_status = 0;
    }
    if (sbs_check_bit(bat_out->operation_status, SBS_OPERATION_STATUS_PF)) {
        I2CCHECK(bq_read_dword(SBS_CMD_PF_STATUS, &bat_out->pf_status));
    }
    else {
        // PF status is 0 since the logical or of the field is 0
        bat_out->pf_status = 0;
    }

    BQ_RETURN_SUCCESS;
}

// ========================================
// BQ40 Init/IO Routines
// ========================================

void bq_clear_soc_leds(void) {
    for (uint8_t led = 0; led < sizeof(BQ_LEDS); led++) {
        gpio_put(BQ_LEDS[led], false);
    }
}

void bq_update_soc_leds(uint8_t soc) {
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

void bq_pulse_wake(void) {
    gpio_put(BMS_WAKE_PIN, 1);
    sleep_ms(BATT_WAKE_PULSE_DURATION_MS);
    gpio_put(BMS_WAKE_PIN, 0);
}

void bq_init() {
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
    uint pio_i2c_program = pio_add_program(BQ_PIO_INST, &i2c_program);
    pio_sm_claim(BQ_PIO_INST, BQ_PIO_SM);
    i2c_program_init(BQ_PIO_INST, BQ_PIO_SM, pio_i2c_program, BMS_SDA_PIN, BMS_SCL_PIN);
}

// ========================================
// BQ40 Commands
// ========================================

bq_error_t bq_emshut_enter(void) {
    // Send the 2 command sequence to enable Manual FET Control Emergency Shutdown
    I2CCHECK(bq_mfg_access_cmd(MFG_CMD_MFC_ENABLE_A));
    BQERRCHECK(bq_check_status_successful());
    I2CCHECK(bq_mfg_access_cmd(MFG_CMD_MFC_ENABLE_B));
    BQERRCHECK(bq_check_status_successful());

    BQ_RETURN_SUCCESS;
}

bq_error_t bq_emshut_exit(void) {
    // TODO: Make sure that MFC_DISABLE won't rocket the BQ40 into undefined states if discharging is disabled

    // send a cleanup reset command
    I2CCHECK(bq_mfg_access_cmd(MFG_CMD_MFC_DISABLE));
    BQERRCHECK(bq_check_status_successful());

    BQ_RETURN_SUCCESS;
}

bq_error_t bq_cycle_count(uint16_t *cycles_out) {
    I2CCHECK(bq_read_word(SBS_CMD_CYCLE_COUNT, cycles_out));
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_read_side_detect(bool *side_det_high_out) {
    uint16_t data;
    I2CCHECK(bq_read_word(SBS_CMD_GPIO_READ, &data));
    *side_det_high_out = sbs_check_bit(data, SBS_GPIO_READ_RH1);
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_read_state_of_health(uint8_t *soh_out) {
    I2CCHECK(bq_read_byte(SBS_CMD_STATE_OF_HEALTH, soh_out));
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_read_capacity(uint8_t scaling_factor, uint32_t *design_capacity, uint32_t *full_charge_capacity,
                            uint32_t *remaining_capacity) {
    uint16_t data;
    I2CCHECK(bq_read_word(SBS_CMD_DESIGN_CAPACITY, &data));
    *design_capacity = ((uint32_t) data) * scaling_factor;
    I2CCHECK(bq_read_word(SBS_CMD_FILTERED_CAPACITY, &data));
    *full_charge_capacity = ((uint32_t) data) * scaling_factor;
    I2CCHECK(bq_read_word(SBS_CMD_REMAINING_CAPACITY, &data));
    *remaining_capacity = ((uint32_t) data) * scaling_factor;
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_read_gauging_status(uint32_t *status_out) {
    *status_out = 0;

    size_t len = sizeof(*status_out);
    I2CCHECK(bq_read_block(SBS_CMD_GAUGING_STATUS, (uint8_t *) status_out, &len));
    if (len != 3) {
        I2CCHECK(PIO_SMBUS_ERR_INVALID_RESP);
    }

    BQ_RETURN_SUCCESS;
}
bq_error_t bq_read_charging_status(uint32_t *status_out) {
    *status_out = 0;

    size_t len = sizeof(*status_out);
    I2CCHECK(bq_read_block(SBS_CMD_CHARGING_STATUS, (uint8_t *) status_out, &len));
    if (len != 3) {
        I2CCHECK(PIO_SMBUS_ERR_INVALID_RESP);
    }

    BQ_RETURN_SUCCESS;
}

bq_error_t bq_dbg_read_sbs_int(uint8_t cmd, enum read_width width, uint32_t *out) {
    uint8_t data8;
    uint16_t data16;

    switch (width) {
    case READ_WIDTH_8:
        I2CCHECK(bq_read_byte(cmd, &data8));
        *out = data8;
        break;
    case READ_WIDTH_16:
        I2CCHECK(bq_read_word(cmd, &data16));
        *out = data16;
        break;
    case READ_WIDTH_32:
        I2CCHECK(bq_read_dword(cmd, out));
        break;
    default:
        BQ_RETURN_ERROR(BQ_ERROR_BAD_DBG_COMMAND);
    }

    BQ_RETURN_SUCCESS;
}

bq_error_t bq_dbg_read_sbs_block(uint8_t cmd, uint8_t *block_out, size_t *len) {
    I2CCHECK(bq_read_block(cmd, block_out, len));
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_dbg_read_mfg_block(uint16_t mfg_cmd, uint8_t *block_out, size_t *len) {
    I2CCHECK(bq_mfg_access_read(mfg_cmd, block_out, len));
    BQERRCHECK(bq_check_status_successful());
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_dbg_mfg_cmd(uint16_t mfg_cmd) {
    I2CCHECK(bq_mfg_access_cmd(mfg_cmd));
    BQERRCHECK(bq_check_status_successful());
    BQ_RETURN_SUCCESS;
}

bq_error_t bq_dbg_df_write(uint16_t addr, const uint8_t *write_buf, size_t len) {
    I2CCHECK(bq_mfg_access_write(addr, write_buf, len));
    sleep_ms(150);
    BQERRCHECK(bq_check_status_successful());
    BQ_RETURN_SUCCESS;
}
