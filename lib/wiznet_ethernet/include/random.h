#ifndef _RANDOM_H
#define _RANDOM_H

#include "pico/stdlib.h"
#include <stdlib.h>

#define ROSC_RANDOMBIT_OFFSET 0x1C

#define yield()

static void seed_random_from_rosc()
{
  uint32_t random = 0x811c9dc5;
  uint8_t next_byte = 0;
  volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

  for (int i = 0; i < 16; i++) {
    for (int k = 0; k < 8; k++) {
      next_byte = (next_byte << 1) | (*rnd_reg & 1);
    }

    random ^= next_byte;
    random *= 0x01000193;
  }

  srand(random);
}

static unsigned int random_range(unsigned int min, unsigned int max){
    return (rand() % (max-min)) + min;
}

#endif