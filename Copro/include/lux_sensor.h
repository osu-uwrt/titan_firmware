#ifndef _LUX_SENSOR_H
#define _LUX_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

bool lux_initialized;
double lux_read(void);
void lux_init(void);

#endif