#ifndef BQ40Z80_CORE1_H
#define BQ40Z80_CORE1_H

#include <stdbool.h>
#include <stdint.h>

#define BQ_ADDR 0x0B
#define PIO_SM 0

#define WARN_SOC_THRESH 60
#define STOP_SOC_THRESH 30

struct core1_battery_status {
    uint16_t voltage;
    int16_t current;
    int16_t avg_current;
    uint16_t time_to_empty;
    uint8_t soc;
    uint32_t safety_status_reg;
    bool port_detected;
};

struct core1_operation_status {
    bool safety_status;
    bool present;
    bool discharge;
};

struct mfg_info {
    uint16_t serial;
    uint8_t mfg_day, mfg_mo;
    uint16_t mfg_year;
    char name[21];
};

enum bq_reg_addr {
    BQ_READ_PACK_VOLT = 0x09,
    BQ_READ_PACK_CURR = 0x0A,
    BQ_READ_AVRG_CURR = 0x0B,
    BQ_READ_RELAT_SOC = 0x0D,
    BQ_READ_TIME_EMPT = 0x11,
    BQ_READ_PACK_STAT = 0x16,
    BQ_READ_GPIO = 0x48,
    BQ_READ_SAFE_STAT = 0x51,
    BQ_READ_OPER_STAT = 0x54,
    BQ_READ_CELL_SERI = 0x1C,
    BQ_READ_CELL_DATE = 0x1B,
    BQ_READ_CELL_NAME = 0x21,
    // BQ_READ_CHRG_STAT = 0x55,
    // BQ_READ_MFG_STATS = 0x57,
};

typedef enum {
    BQ_ERROR_START_UP_TOO_LONG = 1,
    BQ_ERROR_NOT_CONNECTED,  // FIXME redudant with the above?
    BQ_ERROR_I2C_TRANSACTION,
} core1_bq_error_code;

void core1_init(uint8_t expected_serial);

bool core1_get_pack_mfg_info(struct mfg_info *pack_info_out);

void core1_get_battery_status(struct core1_battery_status *status_out);

bool core1_check_present(void);

bool core1_check_safety_status(uint32_t *safe_status_reg);

void core1_open_dsg_temp(const uint32_t open_time_ms);

// TODO bq_pack_side_det_port(), bq_update_soc_leds()
#endif
