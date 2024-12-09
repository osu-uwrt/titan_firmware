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

#define dynamixel_ping_packet_param_len 0
enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id);

#define dynamixel_reboot_packet_param_len 0
enum DXLLibErrorCode dynamixel_create_reboot_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                    size_t packet_buf_size, dynamixel_id id);

#define dynamixel_write_packet_param_hdr_len 2
enum DXLLibErrorCode dynamixel_create_write_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                   size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                   uint8_t *data, size_t data_len);

#define dynamixel_read_paket_param_len 4
enum DXLLibErrorCode dynamixel_create_read_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                  uint16_t length);

//
// This computes a packet_buf_size which can fit the entire packet to be transmitted
/**
 * @brief Computes the worst case size needed for packet_buf passing when creating a packet with dynamixel_create_X
 * functions. This assumes that every byte will have byte stuffing, and includes overhead for other dynamixel packet
 * fields.
 *
 * @param param_len The length of the parameters in the dynamixel packet
 * @return size_t The worst case length of the dynamixel packet (you should make packet_buf this size when creating
 * packets)
 */
static inline size_t dynamixel_calc_packet_worst_case_len(size_t param_len) {
    // 11 bytes is the overhead of the dynamixel packet (this is for the status packet, worst case length, all other 10)
    return 11 + (4 * ((param_len) / 3));
}

#endif
