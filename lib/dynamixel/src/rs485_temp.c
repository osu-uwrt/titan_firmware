#include "async_rs485.h"

void async_rs485_write(uint8_t *data, uint16_t data_len, async_rs485_on_write cb) {
    (void) data;
    (void) data_len;
    (void) cb;
} 

void async_rs485_read(uint8_t *data, uint16_t data_len, async_rs485_on_read cb) {
    (void) data;
    (void) data_len;
    (void) cb;
}