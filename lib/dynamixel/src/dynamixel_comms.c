#include "dxl_packet.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include "async_rs485.h"
#include <assert.h> // TODO: remove

#define PROTOCOL 2

#define DYNAMIXEL_PACKET_BUF_SIZE 128
/** Includes all of the bytes up to the packet length. */
#define DYNAMIXEL_REPLY_HEADER_SIZE 8
static_assert(PROTOCOL == 2); // TODO: support other protocols

#define DXL_RETCHECK(fn) { enum DXLLibErrorCode temp_rc = fn; if((temp_rc != DXL_LIB_OK)){return temp_rc;}}

struct current_request { 
    dynamixel_request_cb user_cb;
    enum DXLInstruction instr;
};

static bool packet_in_flight = false;
static InfoToMakeDXLPacket_t tx_packet;
static InfoToParseDXLPacket_t rx_packet;

/** Stores the current internal request that is happening. */
static struct current_request req;

static uint8_t packet_buf[DYNAMIXEL_PACKET_BUF_SIZE] = {0};
static uint8_t rs485_buf[DYNAMIXEL_PACKET_BUF_SIZE] = {0};

static void on_receive_reply_body(uint8_t *data, uint16_t data_len) { 
    if(data_len == 0) {
        req.user_cb(1, NULL);
    }

    for(int i = 0; i < data_len - 1; i++) {
        if(parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
            req.user_cb(1, NULL);
        }
    }

    if(parse_dxl_packet(&rx_packet, data[data_len - 1]) != DXL_LIB_OK) {
        req.user_cb(1, NULL);
    }

    struct dynamixel_req_result result;
    result.instr = req.instr;
    result.packet = &tx_packet;
    req.user_cb(0, &result);
}

static void on_receive_reply_header(uint8_t error, uint8_t *data, uint8_t *data_len) { 
    begin_parse_dxl_packet(&rx_packet, PROTOCOL, packet_buf, DYNAMIXEL_PACKET_BUF_SIZE);
    uint16_t body_length = 0;

    if(error) {
        req.user_cb(error, NULL);
        return;
    }

    // TODO: Check header
    // TODO: check instruction

    body_length = (data[5] | (data[6] << 8)); 
    assert(body_length + DYNAMIXEL_REPLY_HEADER_SIZE < DYNAMIXEL_PACKET_BUF_SIZE);

    // give the data to the parser.
    for(size_t i = 0; i < data_len; i++) { 
        if(parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
            
        }
    }

    async_rs485_read(rs485_buf + DYNAMIXEL_REPLY_HEADER_SIZE, body_length, on_receive_reply_body);
}

static void on_packet_sent(uint8_t error) { 
    if(error) {
        req.user_cb(error, NULL);
    } else {
        async_rs485_read(rs485_buf, DYNAMIXEL_REPLY_HEADER_SIZE, on_receive_reply_header);
    }
}

/**
 * Sends a packet and recieves a reply. 
 * 
 * The process is as follows: 
 * 
 * 1. A Packet is written
 * 2. The header of the reply is read (fixed amount of bytes). This is used to determine the length of the rest of the packet. 
 * 3. The rest of packet is read and parsed. 
*/
static enum DXLLibErrorCode dynamixel_send_packet(dynamixel_request_cb callback, InfoToMakeDXLPacket_t *packet) { 
    assert(!packet_in_flight);
    packet_in_flight = true;

    uint8_t *bytes = packet->p_packet_buf;
    uint16_t length = packet->generated_packet_length;
    req.user_cb = callback;
    req.instr = packet->inst_idx;

    async_rs485_write(bytes, length, on_packet_sent);
    
    return DXL_LIB_OK;
}

enum DXLLibErrorCode dynamixel_send_ping(int id, dynamixel_request_cb cb) {
    assert(!packet_in_flight);

    DXL_RETCHECK(begin_make_dxl_packet(&tx_packet, id, PROTOCOL, DXL_INST_PING, 0, packet_buf, DYNAMIXEL_PACKET_BUF_SIZE));
    DXL_RETCHECK(end_make_dxl_packet(&tx_packet));
    DXL_RETCHECK(dynamixel_send_packet(cb, &tx_packet));

    return DXL_LIB_OK;
}

void dynamixel_send_factory_reset() { 
    assert(!packet_in_flight);

}

void dynamixel_send_reboot() { 
    assert(!packet_in_flight);

}

void dynamixel_send_write_table() { 
    assert(!packet_in_flight);

}

void dynamixel_send_read_table() { 
    assert(!packet_in_flight);

}

void dynamixel_get_response() { 
    assert(!packet_in_flight);

}


