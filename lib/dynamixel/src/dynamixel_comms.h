#ifndef DYNAMIXEL_COMM_H
#define DYNAMIXEL_COMM_H

#include "dxl_packet.h"
#include "dynamixel.h"
#include <stdint.h>

struct dynamixel_req_result {
  enum DXLInstruction instr;
  InfoToParseDXLPacket_t *packet;
};

typedef void (*dynamixel_request_cb)(enum dynamixel_error ec,
                                     struct dynamixel_req_result *result);

void dynamixel_send_packet(dynamixel_request_cb callback,
                           InfoToMakeDXLPacket_t *packet);

enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet,
                                                  uint8_t *packet_buf,
                                                  dynamixel_id id);

enum DXLLibErrorCode dynamixel_create_write_packet(
    InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, dynamixel_id id,
    uint16_t start_address, uint8_t *data, size_t data_len);

enum DXLLibErrorCode dynamixel_create_read_packet(InfoToMakeDXLPacket_t *packet,
                                                  uint8_t *packet_buf,
                                                  dynamixel_id id,
                                                  uint16_t start_address,
                                                  uint16_t length);

#endif