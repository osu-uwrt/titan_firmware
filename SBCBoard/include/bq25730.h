#pragma once

typedef enum bq25730_state {
    BQ25730_STATE_DISCONNECTED,
    BQ25730_STATE_CONFIGURING,
    BQ25730_STATE_STANDBY,
    BQ25730_STATE_CHARGING,
    BQ25730_STATE_ERROR,
} bq25730_state_t;

typedef enum bq25730_status {
    BQ25730_OK = 0,
    BQ25730_ERR = 1,
} bq25730_status_t;

void bq25730_init(unsigned int busNum);

void bq25730_clear_error();

bq25730_status_t bq25730_wait_until_configured();

bq25730_state_t bq25730_get_state();
