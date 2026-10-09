#include "actuators/claw.h"

#include "actuators/actuator.h"
#include "safety_interface.h"
#include "titan/logger.h"

#include <stdlib.h>

static const int16_t CLAW_MAX_SPEED = 1000;
static const int16_t CLAW_TOLERANCE = 10;

static const int32_t CLAW_CLOSED_POSITION = 0;
static const int32_t CLAW_OPEN_POSITION = 3000;

static const uint32_t FEEDBACK_TIMEOUT_MS = 200;
static const uint32_t MOVE_TIMEOUT_MS = 10000;
static const uint32_t STALL_TIMEOUT_MS = 1000;
static const int32_t PROGRESS_COUNTS = 5;

static claw_t claw;
static bool read_pending;
static bool stop_pending;
static absolute_time_t last_feedback;
static absolute_time_t move_deadline;
static absolute_time_t progress_deadline;
static int32_t progress_position;
static volatile bool disable_requested;
static const char *position_feedback = "No claw position command received";

static bool feedback_fresh(void) {
    return servo_position_is_valid(&claw.position) &&
           absolute_time_diff_us(last_feedback, get_absolute_time()) < FEEDBACK_TIMEOUT_MS * 1000;
}

static void stop_locked(void) {
    claw.moving = false;
    stop_pending = true;
}

static const char *position_rejection_reason(int32_t position) {
    if (position < CLAW_CLOSED_POSITION || position > CLAW_OPEN_POSITION)
        return "Claw position rejected: target must be 0..3000";
    if (disable_requested)
        return "Claw position rejected: safety disable pending";
    if (!claw.servo.desired_armed_state)
        return "Claw position rejected: claw is disarmed; use command/actuator/claw/arm";
    if (!claw.servo.enabled)
        return "Claw position rejected: servo has not confirmed arming";
    if (!feedback_fresh())
        return "Claw position rejected: no valid position feedback within 200 ms";
    if (stop_pending)
        return "Claw position rejected: stop command still pending";
    return NULL;
}

static void claw_position_read_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    read_pending = false;
    if (err != SERVO_READ_OK)
        return;

    const int16_t raw = (int16_t) ((uint16_t) rx_packet.param_buf[0] | (uint16_t) rx_packet.param_buf[1] << 8);

    if (!feedback_fresh()) {
        servo_position_invalidate(&claw.position);
        if (claw.moving) {
            LOG_WARN("Claw stopped: position feedback arrived after freshness timeout");
            stop_locked();
        }
    }
    servo_position_update(&claw.position, raw);
    last_feedback = get_absolute_time();
    claw.servo.connected = true;
    claw.servo.curr_deg = raw < 0 ? 0 : (raw > 1000 ? 240 : raw * 240 / 1000); //don't technically need this but might be good for debugging
}

void claw_init(void) {
    make_servo(&claw.servo, 5, 0);
    stop_pending = true;
}

servo_t *claw_get_servo(void) {
    return &claw.servo;
}

void claw_notify_disable(void) {
    disable_requested = true;
}

static void start_move(void) {
    claw.moving = true;
    move_deadline = make_timeout_time_ms(MOVE_TIMEOUT_MS);
    progress_deadline = make_timeout_time_ms(STALL_TIMEOUT_MS);
    progress_position = servo_position_get(&claw.position);
}

bool claw_set_position(int32_t position) {
    const char *reason = position_rejection_reason(position);
    const bool accepted = reason == NULL;
    position_feedback = accepted ? "Claw position accepted" : reason;
    if (accepted) {
        claw.target_position = position;
        if (!claw.moving)
            start_move();
        LOG_INFO("Claw move accepted: target %ld, current %ld", (long) position,
                 (long) servo_position_get(&claw.position));
    } else {
        LOG_WARN("Claw move rejected: target %ld (allowed 0..3000), armed %u, enabled %u, feedback fresh %u, stop pending %u, disable requested %u",
                 (long) position, (unsigned int) claw.servo.desired_armed_state,
                 (unsigned int) claw.servo.enabled, (unsigned int) feedback_fresh(),
                 (unsigned int) stop_pending, (unsigned int) disable_requested);
    }
    return accepted;
}

const char *claw_get_position_feedback(void) {
    return position_feedback;
}

int32_t claw_get_position(void) {
    const int32_t position = servo_position_get(&claw.position);
    return position;
}

bool claw_get_raw_position(int16_t *position_out) {
    if (!position_out)
        return false;

    const bool valid = feedback_fresh();
    if (valid)
        *position_out = claw.position.absolute_pos;

    return valid;
}

bool claw_get_voltage(uint16_t *voltage_out) {
    return servo_get_voltage(&claw.servo, voltage_out);
}

bool claw_tare(void) {
    const bool accepted = !claw.moving && !stop_pending && feedback_fresh();
    if (accepted) {
        servo_position_tare(&claw.position);
        claw.target_position = CLAW_CLOSED_POSITION;
    }
    return accepted;
}

bool claw_open(void) {
    return claw_set_position(CLAW_OPEN_POSITION);
}

bool claw_close(void) {
    return claw_set_position(CLAW_CLOSED_POSITION);
}

void claw_stop(void) {
    stop_locked();
}

void claw_update(void) {
    if (disable_requested) {
        LOG_WARN("Claw disabled by safety request");
        disable_requested = false;
        stop_locked();
        servo_set_armed(&claw.servo, false);
    }

    if (!feedback_fresh()) {
        claw.servo.connected = false;
        servo_position_invalidate(&claw.position);
        if (claw.moving) {
            LOG_WARN("Claw stopped: position feedback expired");
            stop_locked();
        }
    }

    if (claw.moving && (!claw.servo.desired_armed_state || !claw.servo.enabled)) {
        LOG_WARN("Claw stopped: servo is disarmed");
        stop_locked();
    }

    if (claw.moving) {
        const int32_t current = servo_position_get(&claw.position);
        if (abs(current - progress_position) >= PROGRESS_COUNTS) {
            progress_position = current;
            progress_deadline = make_timeout_time_ms(STALL_TIMEOUT_MS);
        }

        const int32_t error = claw.target_position - current;
        if (time_reached(move_deadline) || time_reached(progress_deadline) || abs(error) <= CLAW_TOLERANCE) {
            if (abs(error) <= CLAW_TOLERANCE)
                LOG_INFO("Claw reached target: current %ld, target %ld", (long) current,
                         (long) claw.target_position);
            else if (time_reached(move_deadline))
                LOG_WARN("Claw stopped: move timed out at %ld, target %ld", (long) current,
                         (long) claw.target_position);
            else
                LOG_WARN("Claw stopped: no encoder progress for %lu ms, current %ld, target %ld",
                         (unsigned long) STALL_TIMEOUT_MS, (long) current, (long) claw.target_position);
            stop_locked();
        } else {
            if (error > CLAW_TOLERANCE)
                servo_set_motor_speed(&claw.servo, CLAW_MAX_SPEED);
            else
                servo_set_motor_speed(&claw.servo, -CLAW_MAX_SPEED);
        }
    }

    if (stop_pending)
        stop_pending = !servo_set_motor_speed(&claw.servo, 0);
    else if (!stop_pending && !read_pending)
        read_pending = servo_read_position(&claw.servo, claw_position_read_cb);
}
