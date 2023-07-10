#include <stdint.h>
#include "driver/async_i2c.h"

#include "ms5837.h"
#include "ms5837_commands.h"

static struct depth_state {
    // --- I2C data ---
    // The async i2c request data
    struct async_i2c_request req;
    // The MS5837 command to transmit
    uint8_t cmd;
    // Buffer for data to be received
    uint8_t rx_buf[3];
    // True if req is in a transaction - Required for async i2c
    volatile bool in_transaction;

    // ---Operation Control Data---
    // True whenever an operation is currently being performed
    volatile bool performing_operation;
    // Contains the success callback after an operation succeeds
    union {
        ms5837_init_cb init_cb;
        ms5837_read_cb read_cb;
    } success_cb;
    // Contains the failure callback after an operation fails
    ms5837_error_cb error_cb;

    // ---Reset operation state---
    // Contains the selected sensor type
    enum depth_sensor_type sensor_type;
    // True if the sensor has been initialized
    bool sensor_initialized;
    // The index of the current byte being read from the PROM during init
    unsigned int prom_read_index;
    // Contains contents of PROM from depth sensor
    // Note: The CRC bits will be zeroed out from the crc calculation
    uint16_t sensor_prom[8];

    // ---Conversion Operation State---
    // False: Reading D1 from sensor, True: Reading D2 from sensor
    bool reading_d2;
    // Variable to hold the d1 reading from the ADC when reading the D2 value from the sensor
    uint32_t d1_temp;
} depth_inst = {0};

/**
 * @brief Aborts the current operation with the specific error event.
 * This handles proper state cleanup and callback firing.
 *
 * @param inst The depth sensor instance to abort
 * @param event The error event that caused the abort
 */
static void ms5837_abort_operation(struct depth_state *inst, enum depth_error_event event) {
    inst->performing_operation = false;
    if (inst->error_cb) {
        inst->error_cb(event);
    }
}

/**
 * @brief Failure callback for any i2c requests. This handles error reporting
 *
 * @param req The request that failed
 * @param abort_data The contents of the abort register
 */
static void ms5837_i2c_error_cb(const struct async_i2c_request* req, __unused uint32_t abort_data) {
    struct depth_state *inst = (struct depth_state*)req->user_data;

    //LOG_DEBUG("Error communicating with depth sensor (Tx Abort: %lu, cmd: %d)", abort_data, inst->cmd);
    if (inst->cmd == DEPTH_CMD_RESET)
        ms5837_abort_operation(inst, DEPTH_ERROR_RESET_CMD);
    else
        ms5837_abort_operation(inst, DEPTH_ERROR_COMM_FAIL);
}

/**
 * @brief Sends the requested I2C command to the ms5837 sensor. This handles all error events during the transfer.
 *
 * @param inst The depth sensor instance
 * @param cmd The ms5837 command to send
 * @param recv_size The number of bytes the command responds with
 * @param completed_cb Callback if the command successfully completes
 */
static void ms5837_i2c_cmd(struct depth_state *inst, uint8_t cmd, size_t recv_size, async_i2c_cb_t completed_cb) {
    invalid_params_if(DEPTH, recv_size > sizeof(inst->rx_buf));

    inst->cmd = cmd;
    inst->req.bytes_to_receive = recv_size;
    inst->req.completed_callback = completed_cb;
    if (!async_i2c_enqueue(&inst->req, &inst->in_transaction)) {
        ms5837_abort_operation(inst, DEPTH_ERROR_I2C_QUEUE_FULL);
    }
}

bool ms5837_busy(void) {
    return depth_inst.performing_operation;
}

// ========================================
// Conversion Functions
// ========================================

/**
 * @brief Does calculations with PROM and ADC readings to calculate the pressure and temperature readings for the MS5837-30BA sensor
 * Taken from the datasheet (And previous python firmware)
 *
 * @param depth_prom The PROM of the sensor
 * @param D1 The ADC reading of the D1 Conversion
 * @param D2 The ADC reading of the D2 Conversion
 * @param pressure_out Pointer to write the pressure from the conversion
 * @param temp_out Pointer to write the temperature from the conversion
 */
static void ms5837_30ba_calculate(uint16_t depth_prom[], uint32_t D1, uint32_t D2, int32_t *pressure_out, int32_t *temp_out) {
    int64_t OFFi = 0;
    int64_t SENSi = 0;
    int Ti = 0;

    int32_t dT = D2-((int32_t)depth_prom[5]) * 256;
    int64_t SENS = ((int64_t)depth_prom[1]) * 32768 + (((int64_t)depth_prom[3]) * dT)/256;
    int64_t OFF = ((int64_t)depth_prom[2])*65536+(((int64_t)depth_prom[4])*dT)/128;
    //int32_t pressure = (D1*SENS/(2097152)-OFF)/(8192);

    int32_t temp = 2000+dT*((int64_t)depth_prom[6])/8388608;

    // Second order compensation

    if ((temp/100) < 20) { // Low temp
        Ti = (3*dT*dT)/(8589934592);
        OFFi = (3*(temp-2000)*(temp-2000))/2;
        SENSi = (5*(temp-2000)*(temp-2000))/8;
        if ((temp/100) < -15) { // Very low temp
            OFFi = OFFi+7*(temp+1500)*(temp+1500);
            SENSi = SENSi+4*(temp+1500)*(temp+1500);
        }
    } else if ((temp/100) >= 20) { // High temp
        Ti = 2*(dT*dT)/(137438953472);
        OFFi = (1*(temp-2000)*(temp-2000))/16;
        SENSi = 0;
    }

    int64_t OFF2 = OFF-OFFi;
    int64_t SENS2 = SENS-SENSi;

    *temp_out = (temp-Ti);
    *pressure_out = (((D1*SENS2)/2097152-OFF2)/8192)/10.0;
}

/**
 * @brief Does calculations with PROM and ADC readings to calculate the pressure and temperature readings for the MS5837-02BA sensor
 * Taken from the datasheet (And previous python firmware)
 *
 * @param depth_prom The PROM of the sensor
 * @param D1 The ADC reading of the D1 Conversion
 * @param D2 The ADC reading of the D2 Conversion
 * @param pressure_out Pointer to write the pressure from the conversion
 * @param temp_out Pointer to write the temperature from the conversion
 */
static void ms5837_02ba_calculate(uint16_t depth_prom[], uint32_t D1, uint32_t D2, int32_t *pressure_out, int32_t *temp_out) {
    // TODO: Fill me out
}


// ========================================
// Initialization Operation
// ========================================

static void ms5837_reset_finished(const struct async_i2c_request *req);
static int64_t ms5837_reset_timer_callback(alarm_id_t id, void *user_data);
static void ms5837_prom_read_finished(const struct async_i2c_request *req);

void ms5837_init(unsigned int bus_id, enum depth_sensor_type sensor_type, ms5837_init_cb init_cb, ms5837_error_cb error_cb) {
    struct depth_state *inst = &depth_inst;

    // Make sure we aren't doing anything before we reinitialize state
    hard_assert_if(DEPTH, depth_inst.performing_operation);

    inst->sensor_type = sensor_type;

    // Set up the constant fields of requst
    inst->req.i2c_num = bus_id;
    inst->req.address = DEPTH_I2C_ADDR;
    inst->req.nostop = false;
    inst->req.tx_buffer = &inst->cmd;
    inst->req.rx_buffer = &inst->rx_buf;
    inst->req.bytes_to_send = 1;
    inst->req.failed_callback = &ms5837_i2c_error_cb;
    inst->req.next_req_on_success = NULL;
    inst->req.user_data = inst;
    inst->req.timeout = nil_time;

    // Begin an operation
    inst->performing_operation = true;
    inst->error_cb = error_cb;
    inst->success_cb.init_cb = init_cb;

    // First part of init is depth sensor reset
    ms5837_i2c_cmd(inst, DEPTH_CMD_RESET, 0, ms5837_reset_finished);
}

/**
 * @brief Calculates CRC4 value from the provided prom
 * Taken from datasheet
 * Note this function expects a prom of 8 bytes, even though the prom is 7 bytes on the sensor
 * This function clears the CRC4 value from the prom, so it must be saved before calling
 *
 * @param n_prom The PROM to calculate the CRC4 for
 * @return unsigned char The calculated CRC for the PROM
 */
static unsigned char crc4(uint16_t n_prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
{
    int cnt; // simple counter
    unsigned int n_rem=0; // crc remainder
    unsigned char n_bit;
    n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7]=0; // Subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    {
        // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    return (n_rem ^ 0x00);
}

/**
 * @brief Callback on completion of the sensor reset command
 *
 * @param req Request that caused the callback
 */
static void ms5837_reset_finished(const struct async_i2c_request *req) {
    struct depth_state *inst = (struct depth_state*)req->user_data;

    if (add_alarm_in_ms(DEPTH_RESET_DELAY_MS, ms5837_reset_timer_callback, inst, true) < 0) {
        ms5837_abort_operation(inst, DEPTH_ERROR_ALARM_QUEUE_FULL);
    }
}

/**
 * @brief Timer callback for the reset delay to allow the sensor time to reset
 * This starts the next stage of initialization
 *
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This contains the sensor instance
 * @return int64_t If/How to restart the timer
 */
static int64_t ms5837_reset_timer_callback(__unused alarm_id_t id, void *user_data) {
    struct depth_state *inst = (struct depth_state*)user_data;

    inst->prom_read_index = 0;
    ms5837_i2c_cmd(inst, DEPTH_CMD_PROM_READ(0), 2, ms5837_prom_read_finished);
    return 0;
}

/**
 * @brief I2C Callback after succesful reading of a byte from the PROM
 *
 * @param req The request which caused the callback
 */
static void ms5837_prom_read_finished(const struct async_i2c_request *req) {
    struct depth_state *inst = (struct depth_state*)req->user_data;

    inst->sensor_prom[inst->prom_read_index] = (inst->rx_buf[0] << 8) + inst->rx_buf[1];
    inst->prom_read_index++;

    if (inst->prom_read_index < 7) {
        ms5837_i2c_cmd(inst, DEPTH_CMD_PROM_READ(inst->prom_read_index), 2, ms5837_prom_read_finished);
    } else {
        uint8_t crc = (inst->sensor_prom[0] >> 12) & 0xF;
        uint8_t calculated_crc = crc4(inst->sensor_prom);
        if (crc != calculated_crc) {
            ms5837_abort_operation(inst, DEPTH_ERROR_BAD_CRC);
            return;
        }

        // If CRC checks okay, we have successfully initialized the sensor
        inst->sensor_initialized = true;
        inst->performing_operation = false;
        inst->success_cb.init_cb();
    }
}


// ========================================
// Conversion Operation
// ========================================

static void ms5837_convert_cmd_finished(const struct async_i2c_request *req);
static int64_t ms5837_adc_wait_callback(alarm_id_t id, void *user_data);
static void ms5837_adc_read_finished(const struct async_i2c_request *req);

void ms5837_do_conversion(ms5837_read_cb read_cb, ms5837_error_cb error_cb) {
    struct depth_state *inst = &depth_inst;

    // Make sure we aren't in an existing operation
    hard_assert_if(DEPTH, depth_inst.performing_operation);
    hard_assert_if(DEPTH, !depth_inst.sensor_initialized);

    // Begin an operation
    inst->performing_operation = true;
    inst->error_cb = error_cb;
    inst->success_cb.read_cb = read_cb;

    // First part of init is depth sensor reset
    inst->reading_d2 = false;
    ms5837_i2c_cmd(inst, DEPTH_CMD_CONVERT_D1(DEPTH_OVERSAMPLING), 0, ms5837_convert_cmd_finished);
}

/**
 * @brief Callback after the begin conversion request is sent
 *
 * @param req The request which caused the callback
 */
static void ms5837_convert_cmd_finished(const struct async_i2c_request *req) {
    struct depth_state *inst = (struct depth_state*)req->user_data;

    if (add_alarm_in_us(DEPTH_CONVERT_DELAY_US(DEPTH_OVERSAMPLING), ms5837_adc_wait_callback, inst, true) < 0) {
        ms5837_abort_operation(inst, DEPTH_ERROR_ALARM_QUEUE_FULL);
    }
}

/**
 * @brief Timer callback to give time for the depth sensor to collect the requested data
 *
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This contains the sensor instance
 * @return int64_t If/How to restart the timer
 */
static int64_t ms5837_adc_wait_callback(__unused alarm_id_t id, void *user_data) {
    struct depth_state *inst = (struct depth_state*)user_data;

    ms5837_i2c_cmd(inst, DEPTH_CMD_ADC_READ, 3, ms5837_adc_read_finished);
    return 0;
}

/**
 * @brief Callback after a successful read of the data in the depth sensor's ADC
 *
 * @param req The request which caused the callback
 */
static void ms5837_adc_read_finished(const struct async_i2c_request *req){
    struct depth_state *inst = (struct depth_state*)req->user_data;

    if (!inst->reading_d2) {
        inst->reading_d2 = true;
        inst->d1_temp = inst->rx_buf[0] << 16 | inst->rx_buf[1] << 8 | inst->rx_buf[2];
        ms5837_i2c_cmd(inst, DEPTH_CMD_CONVERT_D2(DEPTH_OVERSAMPLING), 0, ms5837_convert_cmd_finished);
        return;
    }

    // Reading d2, do the conversion
    uint32_t d2 = inst->rx_buf[0] << 16 | inst->rx_buf[1] << 8 | inst->rx_buf[2];

    // Calculate depth and pressure from calibration and the two conversions
    int32_t pressure, temp;
    if (inst->sensor_type == MS5837_30BA) {
        ms5837_30ba_calculate(inst->sensor_prom, inst->d1_temp, d2, &pressure, &temp);
    }
    else if (inst->sensor_type == MS5837_02BA) {
        ms5837_02ba_calculate(inst->sensor_prom, inst->d1_temp, d2, &pressure, &temp);
    }
    else {
        invalid_params_if(DEPTH, true);
    }

    // Report the conversion data
    inst->performing_operation = false;
    inst->success_cb.read_cb(pressure, temp);
}
