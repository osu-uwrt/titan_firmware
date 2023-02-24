#ifndef _RANDOM_H
#define _RANDOM_H

#include <stdint.h>
#include "hardware/structs/rosc.h"

static inline void seed_random_from_rosc()
{
  uint32_t random = 0x811c9dc5;
  uint8_t next_byte = 0;

  for (int i = 0; i < 16; i++) {
    for (int k = 0; k < 8; k++) {
      next_byte = (next_byte << 1) | (rosc_hw->randombit & 1);
    }

    random ^= next_byte;
    random *= 0x01000193;
  }

  srand(random);
}

static inline unsigned int random_range(unsigned int min, unsigned int max){
    return (rand() % (max-min)) + min;
}

#endif