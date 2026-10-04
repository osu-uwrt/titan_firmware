#include <actuators/servo_position.h>

int32_t servo_position_get(const servo_position_t *pos) {
    return pos->absolute_pos - pos->zero_offset;
}

void servo_position_tare(servo_position_t *pos) {
    pos->zero_offset = pos->absolute_pos;
}

void servo_position_set(servo_position_t *pos, const int32_t raw_pos) {
    pos->absolute_pos = raw_pos;
}

void servo_position_invalidate(servo_position_t *pos) {
    pos->valid = false;
    pos->initialized = false;
}

void servo_position_update(
    servo_position_t *pos, const int16_t raw
) {
    if (!pos->initialized) {
        pos->last_raw_pos = raw;
        pos->initialized = true;
        pos->valid = true;
        return;
    }

    int32_t delta = raw - pos->last_raw_pos;
    if (delta > WRAP_THRESHOLD) {
        delta -= ENCODER_PERIOD;
    } else if (delta < -WRAP_THRESHOLD) {
        delta += ENCODER_PERIOD;
    }

    pos->absolute_pos += delta;
    pos->last_raw_pos = raw;
}

bool servo_position_is_valid(const servo_position_t *pos) {
    return pos->valid;
}
