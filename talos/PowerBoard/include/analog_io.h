#ifndef ANALOG_IO
#define ANALOG_IO
#include "stdint.h"

void analog_io_init();

uint16_t analog_io_read_port_meas();

uint16_t analog_io_read_stbd_meas();

#endif