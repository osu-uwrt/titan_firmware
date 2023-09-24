#ifndef DRIVER__SHT41_H_
#define DRIVER__SHT41_H_

#include <stdbool.h>
#include <stdint.h>
/**
 * @file driver/sht41.h
 *
 * @brief Driver to communicate with SHT41 temperature/humidity sensors.
 *
 */

typedef void (*sht41_on_read_callback)();

typedef union sht41_error {
    // TODO put in error code, error source struct field
    // TODO data field
} sht41_error_t;

typedef void (*sht41_error_cb)(sht41_error_t error);  // FIXME needs inspection

void sht41_init(sht41_error_cb board_error_cb);

bool sht41_is_valid(void);

int16_t sht41_read_temp(void);

int16_t sht41_read_rh(void);
#endif
