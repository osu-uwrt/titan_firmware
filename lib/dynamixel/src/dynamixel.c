
#include "dynamixel.h"
#include <assert.h>

#include "dynamixel_comms.h"

#define MAX_SERVO_CNT 8

enum internal_state {
    UNINITIALIZED,
    /* On startup we want to ping all of the servos */
    STATE_PINGING,
    /** Reading all of the servo positions */
    STATE_READING,
    /** Sending commands to the servos */
    STATE_WRITING,
    /** Dynamixel driver failed */
    STATE_FAILED,
};

enum internal_state state = UNINITIALIZED;

dynamixel_error_cb error_cb;
dynamixel_id servos[MAX_SERVO_CNT];
int servo_cnt;
int current_servo_idx;

static void start_reading() {
    current_servo_idx = 0;
}

static void on_dynamixel_ping(uint8_t error, struct dynamixel_req_result *result) {
    if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    current_servo_idx++;
    
    if(current_servo_idx < servo_cnt) {
        dynamixel_ping(servos[0], on_dynamixel_ping);
    } else {
        start_reading();
    }
}

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb _error_cb) { 
    assert(state == UNINITIALIZED);
    assert(id_cnt < MAX_SERVO_CNT);
    error_cb = _error_cb;
    servo_cnt = id_cnt;

    // Create a copy of the list of IDs
    for(size_t i = 0; i < id_cnt; i++) {
        servos[i] = id_list[i];
    }

    state = STATE_PINGING;
    current_servo_idx = 0;
    dynamixel_ping(servos[0], on_dynamixel_ping);
}