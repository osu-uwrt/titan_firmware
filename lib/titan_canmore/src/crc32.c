#include "titan/canmore/crc32.h"

#define CRC32_POLYNOMIAL 0x04C11DB7

uint32_t crc32_update(const uint8_t *data, size_t len, uint32_t original_crc32) {
    uint32_t crc32 = original_crc32;
    while (len--) {
        crc32 = crc32 ^ ((*data++) << 24);
        for (int i = 0; i < 8; i++) {
            if (crc32 & (1L << 31))
                crc32 = (crc32 << 1) ^ CRC32_POLYNOMIAL;
            else
                crc32 = (crc32 << 1);
        }
    }

    return crc32;
}

uint32_t crc32_compute(const uint8_t *data, size_t len) {
    uint32_t crc32 = 0xFFFFFFFF;
    return crc32_update(data, len, crc32);
}
