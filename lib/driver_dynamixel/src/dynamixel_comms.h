#ifndef DYNAMIXEL_COMM_H
#define DYNAMIXEL_COMM_H

#include "dxl_packet.h"

#include "driver/dynamixel.h"

#include <stdint.h>

// PICO_CONFIG: DYNAMIXEL_PACKET_BUFFER_SIZE, The max number of parameters allowed in a single transfer, min=1, default=128, group=driver_dynamixel
#ifndef DYNAMIXEL_PACKET_BUFFER_SIZE
#define DYNAMIXEL_PACKET_BUFFER_SIZE 128
#endif

struct dynamixel_req_result {
    enum DXLInstruction instr;
    InfoToParseDXLPacket_t *packet;
    uint8_t request_id;
};

typedef void (*dynamixel_request_cb)(dynamixel_error_t ec, struct dynamixel_req_result *result);

void dynamixel_send_packet(dynamixel_request_cb callback, InfoToMakeDXLPacket_t *packet);

enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id);

enum DXLLibErrorCode dynamixel_create_write_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                   size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                   uint8_t *data, size_t data_len);

enum DXLLibErrorCode dynamixel_create_read_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                  uint16_t length);

#endif
