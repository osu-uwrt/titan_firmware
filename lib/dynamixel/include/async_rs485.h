#ifndef ASYNC_RS485_H
#define ASYNC_RS485_H

#include <stdint.h>

// TODO: Move to a seperate library

typedef void (*async_rs485_on_write)(uint8_t error);

typedef void (*async_rs485_on_read)(uint8_t error, uint8_t *data, uint8_t *data_len);

void async_rs485_write(uint8_t *data, uint16_t data_len, async_rs485_on_write cb); 

void async_rs485_read(uint8_t *data, uint16_t data_len, async_rs485_on_read cb);

#endif