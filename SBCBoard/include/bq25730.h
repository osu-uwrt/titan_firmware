#pragma once

void bq25730_init(unsigned int busNum);

void bq25730_start_read_manufacturer_id();

void bq25730_start_read_sys_voltage();

void bq25730_start_write_enable_low_power_mode();

void bq25730_start_write_ADC_option();

void bq25730_start_read_input_voltage();
