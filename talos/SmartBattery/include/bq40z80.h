#ifndef BQ40Z80_H
#define BQ40Z80_H

#include "pico/platform.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef RUNNING_ON_CORE1
#include "bq40z80_reg_map.h"
#endif

// ========================================
// Configuration
// ========================================

#define BQ_PIO_INST pio0
#define BQ_PIO_SM 0
#define BQ_ADDR 0x0B
#define BQ_MAX_ERRORS_BEFORE_DISCONNECT 3

#define BATT_WAKE_PULSE_DURATION_MS 500
#define WARN_SOC_THRESH 60
#define STOP_SOC_THRESH 30

// ========================================
// Public Structs
// ========================================

typedef struct bq_mfg_info_t {
    uint16_t serial;
    uint8_t mfg_day, mfg_mo;
    uint16_t mfg_year;
    uint16_t device_type;
    uint8_t firmware_version[11];
    uint8_t scale_factor;
    uint32_t manufacturing_status;
} bq_mfg_info_t;

// ========================================
// Public Error Reporting
// ========================================

// Negative error codes are PIO_SMBUS errors
// Returned on successful operation
#define BQ_ERROR_SUCCESS 0
// An invalid serial number was read from the device (either all 0s or all 1s)
#define BQ_ERROR_INVALID_SERIAL 1
// Reported if the read serial number does not match the serial number read on first connect
// Either a different BQ40 was connected, or data flash got corrupted
#define BQ_ERROR_SERIAL_CHANGED 2
// Raised if command was issued while the bq40 was in an invalid state
#define BQ_ERROR_INVALID_STATE 3
#define BQ_ERROR_INVALID_MAC_RESP_LEN 4
#define BQ_ERROR_BATT_STATUS_ERROR 5
#define BQ_CHECK_SUCCESSFUL(err) (err.fields.error_code == BQ_ERROR_SUCCESS)

typedef union bq_error {
    struct {
        int8_t error_code;
        uint16_t line;
        uint8_t arg;
    } __packed fields;
    uint32_t data;
} bq_error_t;
static_assert(sizeof(bq_error_t) == sizeof(uint32_t), "Struct did not pack properly");

// This prevents these functions from being called unless the C file sets this define
// Calling these functions from a the main core will break a lot of things
#ifdef RUNNING_ON_CORE1

typedef struct bq_battery_info_t {
    uint32_t operation_status;
    struct sbs_da_status1 da_status1;
    int32_t current;
    int32_t avg_current;
    uint16_t temperature;
    uint8_t max_error;
    uint8_t relative_soc;
    uint16_t time_to_empty;
    uint16_t time_to_full;
    uint16_t battery_status;
    uint16_t charging_voltage;
    uint32_t charging_current;
    bool port_detected;
} bq_battery_info_t;

/**
 * @brief initialize PIO hardware for i2c usage, GPIO for BQ_LEDS_CORE1, and BMS_WAKE_PIN
 */
void bq_init(void);

void bq_update_soc_leds(uint8_t soc);

void bq_pulse_wake(void);

bq_error_t bq_read_mfg_info(bq_mfg_info_t *mfg_out);

bq_error_t bq_read_battery_info(const bq_mfg_info_t *mfg_info, bq_battery_info_t *bat_out);

bq_error_t bq_emshut_enter(void);

bq_error_t bq_emshut_exit(void);

bq_error_t bq_cycle_count(uint16_t *cycles_out);

bq_error_t bq_read_side_detect(bool *side_det_high_out);

bq_error_t bq_read_state_of_health(uint8_t *soh_out);

bq_error_t bq_read_capacity(uint8_t scaling_factor, uint32_t *design_capacity, uint32_t *full_charge_capacity,
                            uint32_t *remaining_capacity);

#endif

#endif
