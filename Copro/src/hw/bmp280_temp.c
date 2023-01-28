#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "basic_logger/logging.h"
#include "build_version.h"

#include "drivers/bmp280.h"
#include "drivers/async_i2c.h"
#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "bmp280_temp"

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

static struct bmp280_dev bmp;
static struct bmp280_uncomp_data ucomp_data;
static absolute_time_t last_reading_valid_time;
static bool last_reading_valid = false;
bool bmp280_initialized = false;

bool bmp280_temp_init(void) {
    i2c_init((SENSOR_I2C == 1 ? i2c1 : i2c0), 200 * 1000);
    gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_SDA_PIN);
    gpio_pull_up(SENSOR_SCL_PIN);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    int8_t rslt;
    struct bmp280_config conf;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_SEC;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);
    if (rslt) goto cleanup;

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);
    if (rslt) goto cleanup;

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;

    /* Pressure over sampling none (disabling pressure measurement) */
    conf.os_pres = BMP280_OS_NONE;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);
    if (rslt) goto cleanup;

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);
    if (rslt) goto cleanup;

    bmp280_initialized = true;
cleanup:
    if (!bmp280_initialized) {
        safety_raise_fault(FAULT_ADC_ERROR);
    }
    i2c_deinit((SENSOR_I2C == 1 ? i2c1 : i2c0));
    return bmp280_initialized;
}

const uint8_t tx_cmd[] = {BMP280_PRES_MSB_ADDR};
uint8_t rx_buf[6];

void get_uncomp_data_cb(const struct async_i2c_request * req) {

    uint8_t *temp = req->rx_buffer;

    struct bmp280_uncomp_data *uncomp_data = &ucomp_data;

    uncomp_data->uncomp_press =
        (int32_t) ((((uint32_t) (temp[0])) << 12) | (((uint32_t) (temp[1])) << 4) | ((uint32_t) temp[2] >> 4));
    uncomp_data->uncomp_temp =
        (int32_t) ((((int32_t) (temp[3])) << 12) | (((int32_t) (temp[4])) << 4) | (((int32_t) (temp[5])) >> 4));
    int8_t rslt = st_check_boundaries((int32_t)uncomp_data->uncomp_temp, (int32_t)uncomp_data->uncomp_press);

    if (rslt == 0) {
        last_reading_valid_time = make_timeout_time_ms(750);
        last_reading_valid = true;
    } else {
        last_reading_valid = false;
    }
}

void get_uncomp_data_fail(__unused const struct async_i2c_request * req, long unsigned int fault_id) {
    LOG_ERROR("Failed to read bp280 sensor: Fault %d", fault_id);
    safety_raise_fault(FAULT_ADC_ERROR);
}

bool in_progress = false;
const struct async_i2c_request get_data_req = {
    .i2c_num = SENSOR_I2C,
    .address = BMP280_I2C_ADDR_SEC,
    .nostop = false,
    .tx_buffer = tx_cmd,
    .rx_buffer = rx_buf,
    .bytes_to_send = 1,
    .bytes_to_receive = 6,
    .completed_callback = get_uncomp_data_cb,
    .failed_callback = get_uncomp_data_fail,
};

bool bmp280_temp_read(double* temp){
    bool valid = false;
    if (last_reading_valid && absolute_time_diff_us(last_reading_valid_time, get_absolute_time()) < 0) {
        int8_t rslt = bmp280_get_comp_temp_double(temp, ucomp_data.uncomp_temp, &bmp);
        if (!rslt) valid=true;
    }

    return valid;
}

static int64_t bmp280_poll_alarm_cb(__unused alarm_id_t id, __unused void *user_data) {
    if (in_progress) {
        LOG_ERROR("BMP280 polling started with one still in progress");
        safety_raise_fault(FAULT_ADC_ERROR);
    }
    async_i2c_enqueue(&get_data_req, &in_progress);

    return 250 * 1000;
}

void bmp280_temp_start_reading(void) {
    hard_assert_if(LIFETIME_CHECK, !bmp280_initialized);
    add_alarm_in_ms(250, &bmp280_poll_alarm_cb, NULL, true);
}









/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    sleep_ms(period_ms);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint8_t *data = malloc(length+1);
    data[0] = reg_addr;
    memcpy(&data[1], reg_data, length);
    int ret = (i2c_write_blocking_until((SENSOR_I2C == 1 ? i2c1 : i2c0), i2c_addr, data, length+1, false, make_timeout_time_ms(50)) == length+1 ? 0 : 1);
    free(data);
    return ret;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    /* Implement the I2C read routine according to the target machine. */
    i2c_write_blocking_until((SENSOR_I2C == 1 ? i2c1 : i2c0), i2c_addr, &reg_addr, 1, false, make_timeout_time_ms(50));
    return (i2c_read_blocking_until((SENSOR_I2C == 1 ? i2c1 : i2c0), i2c_addr, reg_data, length, false, make_timeout_time_ms(50)) == length ? 0 : 1);
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        if (rslt == BMP280_E_NULL_PTR)
        {
            LOG_ERROR("%s\tError [%d] : Null pointer error", api_name, rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            LOG_ERROR("%s\tError [%d] : Bus communication failed", api_name, rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            LOG_ERROR("%s\tError [%d] : Invalid Temperature", api_name, rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            LOG_ERROR("%s\tError [%d] : Device not found", api_name, rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            LOG_ERROR("%s\tError [%d] : Unknown error code", api_name, rslt);
        }
    }
}