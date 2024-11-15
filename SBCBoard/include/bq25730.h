#pragma once

typedef enum bq25730_state {
    /**
     * This is the default state of the driver
     */
    BQ25730_STATE_DISCONNECTED,
    /**
     * The state after the driver is initialized. During this state,
     * the driver will attempt to read the manufacturer ID to verify
     * the chip is succesfully communicating.
     */
    BQ25730_STATE_CONNECTING,
    /**
     * Once the driver verifies the manufacturer ID, it will attempt to configure
     * the chip with the configuration needed for the standby state.
     */
    BQ25730_STATE_CONFIGURING,
    /**
     * This is the state when the chip is configured but not charging.
     * During this state, the driver will repeatedly poll the battery voltage.
     */
    BQ25730_STATE_STANDBY,
    /**
     * This is the state when the driver is configuring the chip to start charging.
     */
    BQ25730_STATE_CHARGING_SETUP,
    /**
     * This is the state when the chip is charging.
     */
    BQ25730_STATE_CHARGING,
    /**
     * This is the state when the driver is configuring the chip to stop charging.
     */
    BQ25730_STATE_STOP_CHARGING,
    /**
     * During this state the driver is considered in error.
     *
     * TODO: if we start an error while charging, we should probably do something?
     */
    BQ25730_STATE_ERROR,
} bq25730_state_t;

typedef enum bq25730_status {
    BQ25730_OK = 0,
    BQ25730_ERR = 1,
} bq25730_status_t;

void bq25730_init(unsigned int busNum);

bq25730_status_t bq25730_start_charging();

void bq25730_clear_error();

bq25730_status_t bq25730_wait_until_configured();

bq25730_state_t bq25730_get_state();

const char *bq25730_state_to_string(bq25730_state_t state);
