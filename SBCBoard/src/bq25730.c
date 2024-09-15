/**
 * @file bq25730.c
 * @brief Driver for the BQ25730 story.
 */

#include "bq25730.h"

typedef enum bq25730_register {
    BQ25730_REG_CHARGER_STATUS = 0x20,
    BQ25730_REG_PROCHOT_STATUS = 0x22,
    BQ25730_REG_IIN_DPM = 0x24,
    // .. //
} bq25730_register_t;

static void on_register_read(bq25730_register_t reg, uint8_t *data, size_t data_len);

static void start_read(bq25730_register_t reg);

static void write_multi(bq25730_register_t reg_start, uint8_t *data. size_t data_len);
