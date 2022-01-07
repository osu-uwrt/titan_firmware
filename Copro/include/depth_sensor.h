#ifndef _DEPTH_SENSOR_H
#define _DEPTH_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DEPTH, Enable/disable assertions in the Depth Sensor module, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_DEPTH
#define PARAM_ASSERTIONS_ENABLED_DEPTH 0
#endif

/**
 * @brief Boolean for if depth is initialized.
 * This will be false until all calibration and zeroing is complete
 * This may never become true if the depth sensor fails to initialize
 */
bool depth_initialized;

/**
 * @brief Reads the value from the depth sensor
 * 
 * @return int32_t 
 */
int32_t depth_read(void);
void depth_init(void);

#endif