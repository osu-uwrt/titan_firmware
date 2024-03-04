#include "canmore/crc32.h"

uint32_t crc32_compute(const uint8_t *data, size_t len) {
    uint32_t crc32 = 0xFFFFFFFF;

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
