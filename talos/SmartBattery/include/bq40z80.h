#ifndef BQ40Z80_H
#define BQ40Z80_H

#include "stdint.h"

#include "pico/stdlib.h"

#define BQ_ADDR 0x0B
#define PIO_SM 0

#define WARN_SOC_THRESH 60
#define STOP_SOC_THRESH 30

typedef struct bq_pack_info_t {
    uint16_t serial;
    uint8_t mfg_day, mfg_mo;
    uint16_t mfg_year;
    char name[21];
} bq_pack_info_t;

enum bq_reg_map {
    BQ_READ_PACK_VOLT = 0x09,
    BQ_READ_PACK_CURR = 0x0A,
    BQ_READ_AVRG_CURR = 0x0B,
    BQ_READ_RELAT_SOC = 0x0D,
    BQ_READ_TIME_EMPT = 0x11,
    BQ_READ_PACK_STAT = 0x16,
    BQ_READ_CELL_DATE = 0x1B,
    BQ_READ_CELL_SERI = 0x1C,
    BQ_READ_CELL_NAME = 0x21,
    BQ_READ_GPIO = 0x48,
    BQ_READ_SAFE_ALRT = 0x50,
    BQ_READ_OPER_STAT = 0x54,
    BQ_READ_CHRG_STAT = 0x55,
    BQ_READ_MFG_STATS = 0x57,
};

enum bq_mac_cmds {
    BQ_MAC_REG_ADDR = 0x0000,
    BQ_MAC_RESET_CMD = 0x0041,
    BQ_MAC_EMG_FET_CTRL_CMD = 0x270C,
    BQ_MAC_EMG_FET_OFF_CMD = 0x043D,
    BQ_MAC_EMG_FET_ON_CMD = 0x23A7,
    BQ_MAC_SHTDN_CMD = 0x0010,
};

// handler for the bq i2c transactions
//  static void bq_handle_i2c_transfer(uint8_t* bq_reg, uint8_t* rx_buf, uint len);

uint8_t bq_init();

// function for reading pack presence of the SBH
uint8_t bq_pack_present();

// function for determining if the discharge fet is closed
uint8_t bq_pack_discharging();

// function for updating the SOC LEDS on the front of the SBH
void bq_update_soc_leds();

// functions for getting readings from the chip

// get the pack state of charge as a value between 100 and 0
uint8_t bq_pack_soc();

// get the current pack voltage in millivolts
uint16_t bq_pack_voltage();

// get the current current reading in signed milliamps
int16_t bq_pack_current();

// get the average cell current draw
int16_t bq_avg_current();

// get the current runtime approximation in minutes
uint16_t bq_time_to_empty();

// get the manufacturing info from the pack
struct bq_pack_info_t bq_pack_mfg_info();

// Check if the side detect pin reports port (pack present must be true for this to be valid)
bool bq_pack_side_det_port();

// used for sending the MAC commands from the mac command enum
void bq_send_mac_command(const uint16_t command);

// used for opening the discharge fets for a specified amount of time
// WARNING THIS WILL BLOCK FOR THE SPECIFIED DURATION as there is a lot of
// important communication with the BQ40z80 during this
void bq_open_dschg_temp(const int64_t open_time_ms);

#endif
