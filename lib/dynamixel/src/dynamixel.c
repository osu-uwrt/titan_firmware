
#include "dynamixel.h"
#include <assert.h>

#include "dynamixel_comms.h"

#define MAX_SERVO_CNT 8
#define MAX_CMDS 32

enum internal_state {
    UNINITIALIZED,
    STATE_PING,
    /* On setup, reads the configuration of each servo. */
    STATE_CONFIG_READ,
    /** Reading all of the servo positions */
    STATE_READING,
    /** Sending commands to the servos */
    STATE_WRITING,
    /** Dynamixel driver failed */
    STATE_FAILED,
};

static uint8_t internal_packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
static InfoToMakeDXLPacket_t internal_packet;

enum internal_state state = UNINITIALIZED;

dynamixel_error_cb error_cb;
dynamixel_id servos[MAX_SERVO_CNT];
int servo_cnt;
int current_servo_idx;

static void start_reading() {
    current_servo_idx = 0;

    // scan through the 
}

static void on_dynamixel_config_read(enum dynamixel_error error, struct dynamixel_req_result *result) {
    if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    current_servo_idx++;
    
    if(current_servo_idx < servo_cnt) {
        // dynamixel_send_ping(servos[0], on_dynamixel_ping);
    } else {
        start_reading();
    }
}

void on_dynamixel_ping(enum dynamixel_error error, struct dynamixel_req_result *result) {
     if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    current_servo_idx++;

     if(current_servo_idx < servo_cnt) {
        if(dynamixel_create_ping_packet(&internal_packet, internal_packet_buf, servos[current_servo_idx]) == DXL_LIB_OK){
            dynamixel_send_packet(on_dynamixel_ping, &internal_packet);
        }        
    } else {
        current_servo_idx = 0;
        // send configure request
    }
}

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb _error_cb) { 
    assert(state == UNINITIALIZED);
    assert(id_cnt < MAX_SERVO_CNT);
    error_cb = _error_cb;
    servo_cnt = id_cnt;

    // copy the list of IDs
    for(size_t i = 0; i < id_cnt; i++) {
        servos[i] = id_list[i];
    }

    state = STATE_PING;
    current_servo_idx = 0;

    if(dynamixel_create_ping_packet(&internal_packet, internal_packet_buf, servos[current_servo_idx]) == DXL_LIB_OK){
        dynamixel_send_packet(on_dynamixel_ping, &internal_packet);
    }
}