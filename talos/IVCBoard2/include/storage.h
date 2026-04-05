#ifndef STORAGE_H
#define STORAGE_H

#include <stddef.h>
#include <stdint.h>

void storage_init();
void storage_read(uint8_t *dest, size_t len);
void storage_flush();
void storage_write(uint8_t *data, size_t len);

#endif  // STORAGE_H
