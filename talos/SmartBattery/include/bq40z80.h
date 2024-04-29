#ifndef BQ40Z80_H
#define BQ40Z80_H

#include "stdint.h"

#include "pico/stdlib.h"

#define BQ_PIO_INST pio0
#define BQ_PIO_SM 0
#define BQ_ADDR 0x0B

#define WARN_SOC_THRESH 60
#define STOP_SOC_THRESH 30

typedef struct bq_mfg_info_t {
    uint16_t serial;
    uint8_t mfg_day, mfg_mo;
    uint16_t mfg_year;
    char name[21];
} bq_mfg_info_t;

typedef struct bq_battery_info_t {
    bool port_detected;
    bool battery_presence;
    int16_t avg_current;
    uint8_t soc;
    bool dsg_mode;
    uint16_t time_to_empty;
    uint16_t voltage;
    int16_t current;
    uint16_t time_to_full;
    uint16_t chg_voltage;
    uint16_t chg_current;
} bq_battery_info_t;

enum bq_reg_map {
    BQ_READ_PACK_VOLT = 0x09,
    BQ_READ_CHG_VOLT = 0x15,
    BQ_READ_PACK_CURR = 0x0A,
    BQ_READ_CHG_CURR = 0x14,
    BQ_READ_AVRG_CURR = 0x0B,
    BQ_READ_RELAT_SOC = 0x0D,
    BQ_READ_TIME_EMPT = 0x11,
    BQ_READ_TIME_FULL = 0x13,
    BQ_READ_PACK_STAT = 0x16,
    BQ_READ_CELL_DATE = 0x1B,
    BQ_READ_CELL_SERI = 0x1C,
    BQ_READ_CELL_NAME = 0x21,
    BQ_MFG_BLK_ACCESS = 0x44,
    BQ_READ_GPIO = 0x48,
    BQ_READ_SAFE_STAT = 0x51,
    BQ_READ_OPER_STAT = 0x54,
    BQ_READ_CHRG_STAT = 0x55,
    BQ_READ_MFG_STATS = 0x57,
    BQ_READ_CYCLE_COUNT = 0x17,
};

enum bq_mac_cmds {
    BQ_MAC_REG_ADDR = 0x0000,
    BQ_MAC_RESET_CMD = 0x0041,
    BQ_MAC_EMG_FET_CTRL_CMD = 0x270C,
    BQ_MAC_EMG_FET_OFF_CMD = 0x043D,
    BQ_MAC_EMG_FET_ON_CMD = 0x23A7,
    BQ_MAC_SHTDN_CMD = 0x0010,
};

// This prevents these functions from being called unless the C file sets this define
// Calling these functions from a the main core will break a lot of things
#ifdef RUNNING_ON_CORE1

/**
 * @brief initialize PIO hardware for i2c usage, GPIO for BQ_LEDS_CORE1, and BMS_WAKE_PIN
 */
void bq40z80_init(void);

void bq40z80_update_soc_leds(uint8_t soc);

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
bool bq40z80_refresh_reg(uint8_t sbh_mcu_serial, bool read_once, bq_battery_info_t *bat_out, bq_mfg_info_t *mfg_out);

// used for opening the discharge fets for a specified amount of time
// WARNING THIS WILL BLOCK FOR THE SPECIFIED DURATION as there is a lot of
// important communication with the BQ40z80 during this
bool bq_open_dschg_temp(const int64_t open_time_ms);

bool bq_cycle_count(uint16_t *cycles_out);

#endif

#endif
