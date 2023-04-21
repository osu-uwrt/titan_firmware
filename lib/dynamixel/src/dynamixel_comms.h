#ifndef DYNAMIXEL_COMM_H
#define DYNAMIXEL_COMM_H

#include <stdint.h>
#include "dynamixel.h"
#include "dxl_packet.h"

struct dynamixel_req_result { 
    enum DXLInstruction instr;
    InfoToParseDXLPacket_t *packet;
};

typedef void (*dynamixel_request_cb)(enum dynamixel_error ec, struct dynamixel_req_result *result);

void dynamixel_send_packet(dynamixel_request_cb callback, InfoToMakeDXLPacket_t *packet);

enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet);

enum DXLLibErrorCode dynamixel_send_ping(); 

void dynamixel_send_factory_reset();

void dynamixel_send_reboot();

void dynamixel_send_write_table();

void dynamixel_send_read_table();

#endif