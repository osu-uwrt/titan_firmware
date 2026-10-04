#include "actuators/claw.h"

#include "actuators/actuator.h"
#include "safety_interface.h"

#include <stdlib.h>

static const float CLAW_KP = 0.5f;
static const int16_t CLAW_MAX_SPEED = 500;
static const int16_t CLAW_MIN_SPEED = 100;
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

static bool feedback_fresh(void) {
    return servo_position_is_valid(&claw.position) &&
           absolute_time_diff_us(last_feedback, get_absolute_time()) < FEEDBACK_TIMEOUT_MS * 1000;
}

static void stop_locked(void) {
    claw.moving = false;
    stop_pending = true;
}

static bool can_move(void) {
    return !disable_requested && claw.servo.desired_armed_state &&
           claw.servo.enabled && feedback_fresh() && !stop_pending;
}

static void claw_position_read_cb(ServoPacket_t rx_packet, enum servo_read_err err) {
    read_pending = false;
    if (err != SERVO_READ_OK)
        return;

    const int16_t raw = (int16_t) ((uint16_t) rx_packet.param_buf[0] | (uint16_t) rx_packet.param_buf[1] << 8);

    if (!feedback_fresh()) {
        servo_position_invalidate(&claw.position);
        if (claw.moving)
            stop_locked();
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
    const bool accepted = can_move() && position >= CLAW_CLOSED_POSITION && position <= CLAW_OPEN_POSITION;
    if (accepted) {
        claw.target_position = position;
        if (!claw.moving)
            start_move();
    }
    return accepted;
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
        *position_out = claw.position.last_raw_pos;

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
        disable_requested = false;
        stop_locked();
        servo_set_armed(&claw.servo, false);
    }

    if (!feedback_fresh()) {
        claw.servo.connected = false;
        servo_position_invalidate(&claw.position);
        if (claw.moving)
            stop_locked();
    }

    if (claw.moving && (!claw.servo.desired_armed_state || !claw.servo.enabled)) {
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
            stop_locked();
        } else {
            int32_t speed = (int32_t) (error * CLAW_KP);
            if (speed > CLAW_MAX_SPEED)
                speed = CLAW_MAX_SPEED;
            else if (speed < -CLAW_MAX_SPEED)
                speed = -CLAW_MAX_SPEED;
            if (speed > 0 && speed < CLAW_MIN_SPEED)
                speed = CLAW_MIN_SPEED;
            else if (speed < 0 && speed > -CLAW_MIN_SPEED)
                speed = -CLAW_MIN_SPEED;

            servo_set_motor_speed(&claw.servo, speed);
        }
    }

    if (stop_pending)
        stop_pending = !servo_set_motor_speed(&claw.servo, 0);
    else if (!stop_pending && !read_pending)
        read_pending = servo_read_position(&claw.servo, claw_position_read_cb);
}
