
#include "dynamixel.h"
#include <assert.h>
#include "basic_queue/queue.h"
#include "dynamixel_controls.h"
#include "dynamixel_comms.h"

#include "pico/time.h"

#define MAX_SERVO_CNT 8
#define MAX_CMDS 8

enum internal_state {
    UNINITIALIZED,
    STATE_PING,
    /* On setup, reads the configuration of each servo. */
    STATE_EEPROM_READ,
    /** Reading all of the servo positions */
    STATE_READING,
    /** Sending commands to the servos */
    STATE_WRITING,
    /** Dynamixel driver failed */
    STATE_FAILED,
};

struct internal_cmd { 
    InfoToMakeDXLPacket_t packet;
    uint8_t packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
};

static struct QUEUE_DEFINE(struct internal_cmd, MAX_CMDS) cmd_queue = {0};

static uint8_t internal_packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
static InfoToMakeDXLPacket_t internal_packet;

enum internal_state state = UNINITIALIZED;

dynamixel_error_cb error_cb;
dynamixel_id servos[MAX_SERVO_CNT];
int servo_cnt;
int current_servo_idx;

static void send_next_cmd();
static void start_reading();

static void send_eeprom_read();

/* ======================== 
*  INTERNAL CMDS
*  ========================
*/

static void on_internal_cmd_complete(enum dynamixel_error error, struct dynamixel_req_result *result) {
    if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    (void) result;

    send_next_cmd();
}

static void send_next_cmd() {
    if (QUEUE_EMPTY(&cmd_queue)) {
        start_reading();
        return;
    }

    state = STATE_WRITING;

    struct internal_cmd *entry = QUEUE_CUR_READ_ENTRY(&cmd_queue);
    dynamixel_send_packet(on_internal_cmd_complete, &entry->packet);
    QUEUE_MARK_READ_DONE(&cmd_queue);
}

static int64_t alarm_send_cmds(alarm_id_t id, void *user_data) {
    (void) id;
    (void) user_data;

    send_next_cmd();

    return 0;
}

static void start_reading() {
    state = STATE_READING;
    current_servo_idx = 0;

    // TODO: actually read here lol 
    hard_assert(add_alarm_in_ms(10, alarm_send_cmds, NULL, true) > 0);
}

/* ======================== 
*  EEPROM READ
*  ========================
*/

static void on_dynamixel_eeprom_read(enum dynamixel_error error, struct dynamixel_req_result *result) {
    if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    current_servo_idx++;
    
    if(current_servo_idx < servo_cnt) {
        send_eeprom_read();
    } else {
        start_reading();
    }
}

static void send_eeprom_read() { 
    
}

/* ======================== 
*  PING
*  ========================
*/

static void on_dynamixel_ping(enum dynamixel_error error, struct dynamixel_req_result *result) {
     if(error) {
        state = STATE_FAILED;
        error_cb(error);
        return;
    }

    (void) result;
    current_servo_idx++;

     if(current_servo_idx < servo_cnt) {
        if(dynamixel_create_ping_packet(&internal_packet, internal_packet_buf, servos[current_servo_idx]) == DXL_LIB_OK){
            dynamixel_send_packet(on_dynamixel_ping, &internal_packet);
        }        
    } else {
        current_servo_idx = 0;
        send_eeprom_read();
    }
}

/* ======================== 
*  USER FUNCTIONS BELOW
*  ========================
*/

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

bool dynamixel_set_id(dynamixel_id old, dynamixel_id new) {
    if (QUEUE_FULL(&cmd_queue)) {
        return false;
    }

    struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&cmd_queue);
    dynamixel_create_write_packet(&entry->packet, entry->packet_buf, old, DYNAMIXEL_CTRL_TABLE_ID_ADDR, &new, 1);
    QUEUE_MARK_WRITE_DONE(&cmd_queue);
    
    return true;
}
