#include "dxl_packet.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include <assert.h> // TODO: 

#define DYNAMIXEL_PACKET_BUF_SIZE 128
#define PROTOCOL 2

#define DXL_RETCHECK(fn) { enum DXLLibErrorCode temp_rc = fn; if((temp_rc != DXL_LIB_OK)){return temp_rc;}}

static bool packet_in_flight = false;
static InfoToMakeDXLPacket_t tx_packet;
static InfoToParseDXLPacket_t rx_packet;

static uint8_t packet_buf[DYNAMIXEL_PACKET_BUF_SIZE] = {0};

static void on_receive_reply_body(uint8_t *data, uint16_t data_len) { 
    if(data_len == 0) {
        // TODO: handle error
    }

    for(int i = 0; i < data_len - 1; i++) {
        if(parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
            // TODO: handle error
        }
    }

    if(parse_dxl_packet(&rx_packet, data[data_len - 1]) != DXL_LIB_OK) {
        // TODO: handle error
    }

    // we read packet good job :) 
    // TODO: call callback
}

static uint16_t on_receive_reply_header(uint8_t *data, uint16_t data_len) { 
    begin_parse_dxl_packet(&rx_packet, PROTOCOL, packet_buf, DYNAMIXEL_PACKET_BUF_SIZE);
    uint16_t body_length = 0;

    // parse packet
    // check header 
    // get body length

    // give the data to the parser.
    for(size_t i = 0; i < data_len; i++) { 
        if(parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
            // TODO: handle error
        }
    }

    return body_length;   
}

static enum DXLLibErrorCode dynamixel_send_packet(dynamixel_on_packet_complete callback, InfoToMakeDXLPacket_t *packet) { 
    assert(!packet_in_flight);
    packet_in_flight = true;

    uint8_t *bytes = packet->p_packet_buf;
    uint16_t length = packet->generated_packet_length;

    // write bytes
    // callback for response header
    // I tell robert the length of the rest of the packet
    // Then robert gives me the rest of the packet
    // callback 
    
    callback(0);
}

static void on_receive_ping() {
    // parse the packet: https://emanual.robotis.com/docs/en/dxl/protocol2/#packet-parameters
}

enum DXLLibErrorCode dynamixel_send_ping(int id) {
    assert(!packet_in_flight);

    DXL_RETCHECK(begin_make_dxl_packet(&tx_packet, id, PROTOCOL, DXL_INST_PING, 0, packet_buf, DYNAMIXEL_PACKET_BUF_SIZE));
    DXL_RETCHECK(end_make_dxl_packet(&tx_packet));
    DXL_RETCHECK(dynamixel_send_packet(NULL, &tx_packet));

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


