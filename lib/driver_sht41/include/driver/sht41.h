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

struct sht41_data {
    int16_t temp;
    int16_t rh;
};

typedef void (*sht41_on_read_callback)();

struct sht41_read_req {
    uint8_t i2c_num;
    struct sht41_data *data;
    sht41_on_read_callback callback;
    void *user_data;
};

bool sht41_init(/* ADD IN ERROR CALLBACK */);

bool sht41_read_async(struct sht41_read_req *req);

#endif
