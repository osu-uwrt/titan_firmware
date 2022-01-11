#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "drivers/async_i2c.h"
#include "drivers/safety.h"
#include "hw/depth_sensor.h"
#include "hw/depth_sensor_commands.h"

#define DEPTH_POLLING_RATE_MS 50

bool depth_initialized = false;
static bool in_transaction = false;

static void depth_begin_zero_depth(void);

// ========================================
// Initialization Code
// ========================================

/**
 * @brief Contains contents of PROM from depth sensor
 * Note: The CRC bits will be zeroed out from the crc calculation
 */
static uint16_t depth_prom[8];

/**
 * @brief The index of the current byte being read from the PROM during init
 */
static int depth_prom_read_index;

/**
 * @brief Failure callback for any initialization requests
 * 
 * @param req The request that failed
 * @param abort_data The contents of the abort register
 */
static void depth_init_failure(const struct async_i2c_request *req, uint32_t abort_data) {
    printf("Failed to init depth sensor (Tx Abort: %d)\n", abort_data);
    safety_raise_fault(FAULT_DEPTH_INIT_ERROR);
}

/**
 * @brief Timer callback for the reset delay to allow the sensor time to reset
 * This starts the next stage of initialization
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t depth_reset_timer_callback(alarm_id_t id, void *user_data) {
    depth_prom_read_index = 0;
    async_i2c_enqueue(&prom_read_req, &in_transaction);
    return 0;
}

/**
 * @brief Callback on completion of the sensor reset command
 * 
 * @param req Request that caused the callback
 */
static void depth_reset_finished(const struct async_i2c_request *req) {
    assert(add_alarm_in_ms(10, depth_reset_timer_callback, NULL, true) > 0);
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
 * @brief I2C Callback after succesful reading of a byte from the PROM
 * 
 * @param req The request which caused the callback
 */
static void depth_prom_read_finished(const struct async_i2c_request *req) {
    depth_prom[depth_prom_read_index] = (prom_read_data[0] << 8) + prom_read_data[1];
    depth_prom_read_index++;

    if (depth_prom_read_index < 7) {
        prom_read_cmd[0] = DEPTH_CMD_PROM_READ(depth_prom_read_index);
        async_i2c_enqueue(&prom_read_req, &in_transaction);
    } else {
        if ((depth_prom[0] & DEPTH_PROM_ID_MASK) != DEPTH_PROM_EXPECTED_ID) {
            printf("Depth Init Error - Invalid PROM Byte 0: %d\n", depth_prom[0]);
            safety_raise_fault(FAULT_DEPTH_INIT_ERROR);
            return;
        }

        uint8_t crc = (depth_prom[0] >> 12) & 0xF;
        uint8_t calculated_crc = crc4(depth_prom);
        if (crc != calculated_crc) {
            printf("Depth Init Error - Invalid CRC: %d expected, %d calculated\n", crc, calculated_crc);
            safety_raise_fault(FAULT_DEPTH_INIT_ERROR);
            return;
        }

        // Everything checks out, now good to begin calibration
        depth_begin_zero_depth();
    }
}


// ========================================
// Reading Code
// ========================================
/**
 * @brief The last pressure reading from the sensor in tens of mbar
 */
static int32_t depth_pressure = 0;
/**
 * @brief The last temperature reading from the sensor in hundreds of deg C
 */
static int32_t depth_temp;


// State management varaibles for the active depth reading command
/**
 * @brief If a read of the depth sensor is in progress
 */
static bool depth_read_running = false;

/**
 * @brief The number of reads remaining in the current read request
 */
static int depth_read_num_reads_remaining;

/**
 * @brief Boolean for what value the request
 * False: Reading D1 from sensor
 * True: Reading D2 from sensor
 */
static bool depth_read_reading_d2;

/**
 * @brief Callback for when the current read request has completed all of the requested reads
 */
static void (*depth_read_finished_cb)(void);

/**
 * @brief Variable to hold the d1 reading from the ADC when reading the D2 value from the sensor
 */
static uint32_t depth_read_d1_temp;



/**
 * @brief Does calculations with PROM and ADC readings to calculate the pressure and temperature readings
 * Taken from the datasheet (And previous python firmware)
 * 
 * @param D1 The ADC reading of the D1 Conversion 
 * @param D2 The ADC reading of the D2 Conversion
 */
static void depth_calculate(uint32_t D1, uint32_t D2) {
    int64_t OFFi = 0;
    int64_t SENSi = 0;
    int Ti = 0;

    int32_t dT = D2-((int32_t)depth_prom[5]) * 256;
    int64_t SENS = ((int64_t)depth_prom[1]) * 32768 + (((int64_t)depth_prom[3]) * dT)/256;
    int64_t OFF = ((int64_t)depth_prom[2])*65536+(((int64_t)depth_prom[4])*dT)/128;
    depth_pressure = (D1*SENS/(2097152)-OFF)/(8192);

    depth_temp = 2000+dT*((int64_t)depth_prom[6])/8388608;

    // Second order compensation

    if ((depth_temp/100) < 20) { // Low temp
        Ti = (3*dT*dT)/(8589934592);
        OFFi = (3*(depth_temp-2000)*(depth_temp-2000))/2;
        SENSi = (5*(depth_temp-2000)*(depth_temp-2000))/8;
        if ((depth_temp/100) < -15) { // Very low temp
            OFFi = OFFi+7*(depth_temp+1500)*(depth_temp+1500);
            SENSi = SENSi+4*(depth_temp+1500)*(depth_temp+1500);
        }
    } else if ((depth_temp/100) >= 20) { // High temp
        Ti = 2*(dT*dT)/(137438953472);
        OFFi = (1*(depth_temp-2000)*(depth_temp-2000))/16;
        SENSi = 0;
    }

    int64_t OFF2 = OFF-OFFi;
    int64_t SENS2 = SENS-SENSi;

    depth_temp = (depth_temp-Ti);
    depth_pressure = (((D1*SENS2)/2097152-OFF2)/8192)/10.0;
}

/**
 * @brief Timer callback to give time for the depth sensor to collect the requested data
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t depth_adc_wait_callback(alarm_id_t id, void *user_data) {
    async_i2c_enqueue(&adc_read_req, &in_transaction);
}

/**
 * @brief Callback after the begin conversion request is sent
 * 
 * @param req The request which caused the callback
 */
static void depth_convert_cmd_finished(const struct async_i2c_request *req) {
    assert(add_alarm_in_ms((int)(2.5e-3 * (1<<(8+DEPTH_OVERSAMPLING))) + 2, &depth_adc_wait_callback, NULL, true) > 0);
}

/**
 * @brief Callback after a successful read of the data in the depth sensor's ADC
 * 
 * @param req The request which caused the callback
 */
static void depth_adc_read_finished(const struct async_i2c_request *req){
    if (depth_read_reading_d2) {
        // Do final processing
        uint32_t d2 = adc_read_data[0] << 16 | adc_read_data[1] << 8 | adc_read_data[2];
        depth_calculate(depth_read_d1_temp, d2);

        depth_read_num_reads_remaining--;
        if (depth_read_num_reads_remaining) {
            depth_read_reading_d2 = false;
            async_i2c_enqueue(&d1_convert_req, &in_transaction);
        } else {
            depth_read_running = false;

            if (depth_read_finished_cb) {
                (*depth_read_finished_cb)();
            }
        }
    } else {
        depth_read_reading_d2 = true;
        depth_read_d1_temp = adc_read_data[0] << 16 | adc_read_data[1] << 8 | adc_read_data[2];
        async_i2c_enqueue(&d2_convert_req, &in_transaction);
    }
}

/**
 * @brief Failure callback for read requests from the sensor
 * 
 * @param req The request that failed
 * @param abort_data The contents of the abort register
 */
static void depth_read_failure(const struct async_i2c_request *req, uint32_t abort_data) {
    printf("Failed to read depth sensor (Tx Abort: %d)", abort_data);
    if (!depth_initialized) {
        // This callback could occur during calibration which would fail to initialize the sensor
        safety_raise_fault(FAULT_DEPTH_INIT_ERROR);
    } else {
        safety_raise_fault(FAULT_DEPTH_ERROR);
    }

    depth_read_running = false;
}

/**
 * @brief Common handler function to start a sequence of sensor reads
 * Only one sensor read can occur at a time
 * 
 * @param num_reads The number of reads to perform
 * @param callback Optional callback on success of performing all of the reads
 */
static void depth_adc_queue_reads(int num_reads, void (*callback)(void)) {
    // If the depth is reading but this function should never be called
    // Since this is an internal function, this is an internal error
    hard_assert_if(DEPTH, depth_read_running);  

    depth_read_running = true;
    depth_read_num_reads_remaining = num_reads;
    depth_read_reading_d2 = false;
    depth_read_finished_cb = callback;
    async_i2c_enqueue(&d1_convert_req, &in_transaction);
}

/**
 * @brief Alarm callback to poll the depth sesnor during operation
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t depth_read_alarm_callback(alarm_id_t id, void *user_data) {
    if (depth_read_running) {
        printf("Depth new transaction started with one still in progress\n");
        safety_raise_fault(FAULT_DEPTH_ERROR);
    } else {
        depth_adc_queue_reads(1, NULL);
    }

    return DEPTH_POLLING_RATE_MS * 1000;
}

// ========================================
// Calibration Code
// ========================================

#define FLUID_DENSITY 997
/**
 * @brief The surface pressure in tens of mbar
 * Generated during calibration
 */
static int32_t surface_pressure;

/**
 * @brief The number of times a zero reading has been added
 */
static int zero_count;

/**
 * @brief Callback after a succesful reading signaling to update zeroing
 */
static void depth_zero_depth(void) {
    if (zero_count == 1) {
        surface_pressure = depth_pressure;
    }
    else if (zero_count > 1) {
        surface_pressure = surface_pressure * .7 + depth_pressure * .3;
    }
    zero_count++;
    if (zero_count < 20) {
        depth_adc_queue_reads(2, &depth_zero_depth);
    } else {
        depth_initialized = true;
        assert(add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth_read_alarm_callback, NULL, true) > 0);
        // TODO: Save surface pressure in watchdog scratch register
    }
}

/**
 * @brief Begins zeroing of the depth sensor
 * Only call as part of the init process
 */
static void depth_begin_zero_depth(void) {
    zero_count = 0;
    depth_adc_queue_reads(20, &depth_zero_depth);
}

// ========================================
// Public Functions
// ========================================

double depth_read(void) {
    hard_assert_if(DEPTH, !depth_initialized);

    return ((depth_pressure - surface_pressure)*100)/(FLUID_DENSITY*9.80665);
}

void depth_init(void) {
    async_i2c_enqueue(&reset_req, &in_transaction);
}