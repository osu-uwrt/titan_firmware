#ifndef ANALOG_IO
#define ANALOG_IO
#include "stdint.h"

void analog_io_init();

float analog_io_read_port_meas();

float analog_io_read_stbd_meas();

#endif