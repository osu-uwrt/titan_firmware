#ifndef HBRIDGE_BOARD_SERVO_POSITION_H
#define HBRIDGE_BOARD_SERVO_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_PERIOD 1500
#define WRAP_THRESHOLD 500

typedef struct {
    int32_t absolute_pos;
    int16_t last_raw_pos;

    int32_t zero_offset;

    bool initialized;
    bool valid;
} servo_position_t;

void servo_position_update(
    servo_position_t *pos,
    int16_t raw
);

int32_t servo_position_get(
    const servo_position_t *pos
);

void servo_position_tare(
    servo_position_t *pos
);

void servo_position_set(
    servo_position_t *pos,
    int32_t raw_pos
);

void servo_position_invalidate(
    servo_position_t *pos
);

bool servo_position_is_valid(const servo_position_t *pos);

#endif  // HBRIDGE_BOARD_SERVO_POSITION_H
