#ifndef DYNAMIXEL_COMM_H
#define DYNAMIXEL_COMM_H

#include <stdint.h>
#include "dxl_packet.h"

struct dynamixel_req_result { 
    enum DXLInstruction instr;
    struct InfoToParseDXLPacket *packet;
};

typedef void (*dynamixel_request_cb)(uint8_t error_code, struct dynamixel_req_result *result);

enum DXLLibErrorCode dynamixel_send_ping(); 

void dynamixel_send_factory_reset();

void dynamixel_send_reboot();

void dynamixel_send_write_table();

void dynamixel_send_read_table();

#endif