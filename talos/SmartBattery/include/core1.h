#ifndef CORE1_H
#define CORE1_H
#include "bq40z80.h"

#include <stdbool.h>
#include <stdint.h>

#define BQ_ADDR 0x0B
#define PIO_SM 0

#define WARN_SOC_THRESH 60
#define STOP_SOC_THRESH 30

void core1_init(uint8_t expected_serial);

bool core1_get_pack_mfg_info(bq_mfg_info_t *pack_info_out);

bool core1_check_present(void);

bool core1_check_port_detected(void);

bool core1_dsg_mode(void);

int16_t core1_avg_current(void);

uint8_t core1_soc(void);

uint16_t core1_time_to_empty(void);

uint16_t core1_time_to_full(void);

uint16_t core1_voltage(void);

int16_t core1_current(void);

void core1_open_dsg_temp(const uint32_t open_time_ms);
#endif
