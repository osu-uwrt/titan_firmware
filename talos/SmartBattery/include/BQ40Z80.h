#ifndef BQ40Z80_H
#define BQ40Z80_H

#include "stdint.h"

#define BQ_ADDR 0x0B
#define PIO_SM 0

typedef struct bq_data_t {
    uint16_t serial;
    float last_soc;
    float last_volt;
};

enum bq_reg_map {
    BQ_READ_PACK_VOLT = 0x09,
    BQ_READ_PACK_CURR = 0x0A,
    BQ_READ_AVRG_CURR = 0x0B,
    BQ_READ_RELAT_SOC = 0x0D,
    BQ_READ_TIME_EMPT = 0x11,
    BQ_READ_PACK_STAT = 0x16,
    BQ_READ_CELL_DATE = 0x1c,
    BQ_READ_CELL_SERI = 0x1c,
    BQ_READ_CELL_NAME = 0x21,
    BQ_READ_SAFE_ALRT = 0x50,
    BQ_READ_OPER_STAT = 0x54,
    BQ_READ_CHRG_STAT = 0x54,
};

// bq_data_t bq_ref;

uint8_t bq_init();

uint8_t bq_pack_present();

void bq_update_soc_leds();

#endif