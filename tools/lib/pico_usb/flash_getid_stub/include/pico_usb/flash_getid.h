#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef union {
    uint8_t data[12];  // Flash ID is 8 bytes, JEDEC ID is 3 bytes
    struct __attribute__((__packed__)) {
        uint64_t flash_id;
        uint32_t
            jedec_id;  // Note JEDEC ID is 3 bytes, relying on initial zeroing of this variable to fill the upper 8 bits
    } info;
} flash_info_t;

extern unsigned char flash_getid_bin[];
extern unsigned int flash_getid_bin_len;

#ifdef __cplusplus
}
#endif
