#include "dynamixel_comms.h"
#include "dxl_packet.h"
#include "dynamixel_controls.h"
#include "string.h"
#include "dynamixel/dynamixel.h"
#include "dxl_packet.h"
#include "dynamixel/async_uart.h"
#include <string.h>
#include <assert.h> // TODO: remove

#define PROTOCOL 2

#define DYNAMIXEL_PACKET_BUF_SIZE 128
/** Includes all of the bytes up to the packet length. */
#define DYNAMIXEL_REPLY_HEADER_SIZE 8
static_assert(PROTOCOL == 2); // TODO: support other protocols

#define DXL_RETCHECK(fn)                                                       \
  {                                                                            \
    enum DXLLibErrorCode temp_rc = fn;                                         \
    if ((temp_rc != DXL_LIB_OK)) {                                             \
      return temp_rc;                                                          \
    }                                                                          \
  }

struct current_request {
  dynamixel_request_cb user_cb;
  enum DXLInstruction instr;
};

static bool packet_in_flight = false;
static InfoToParseDXLPacket_t rx_packet;

/** Stores the current internal request that is happening. */
static struct current_request req;
/** Used internally as the read/write buffer. */
static uint8_t rs485_buf[DYNAMIXEL_PACKET_BUF_SIZE] = {0};

static void on_receive_reply_body(enum async_uart_rx_err error, uint8_t *data, size_t data_len) {
  if (error != ASYNC_UART_RX_OK) {
    req.user_cb(DYNAMIXEL_TTL_ERROR, NULL);
  }

  for (size_t i = 0; i < data_len - 1; i++) {
    if (parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
      req.user_cb(DYNAMIXEL_PACKET_ERROR, NULL);
    }
  }

  if (parse_dxl_packet(&rx_packet, data[data_len - 1]) != DXL_LIB_OK) {
    req.user_cb(DYNAMIXEL_PACKET_ERROR, NULL);
  }

  struct dynamixel_req_result result;
  result.instr = req.instr;
  result.packet = &rx_packet;
  req.user_cb(0, &result);
}

static void on_receive_reply_header(enum async_uart_rx_err error, uint8_t *data, size_t data_len) {
  begin_parse_dxl_packet(&rx_packet, PROTOCOL, rs485_buf,
                         DYNAMIXEL_PACKET_BUF_SIZE);
  uint16_t body_length = 0;

  if (error != ASYNC_UART_RX_OK) {
    req.user_cb(DYNAMIXEL_TTL_ERROR, NULL);
    return;
  }

  // TODO: Check header
  // TODO: check instruction

  body_length = (data[5] | (data[6] << 8));
  assert(body_length + DYNAMIXEL_REPLY_HEADER_SIZE < DYNAMIXEL_PACKET_BUF_SIZE);

  // give the data to the parser.
  for (size_t i = 0; i < data_len; i++) {
    if (parse_dxl_packet(&rx_packet, data[i]) != DXL_LIB_PROCEEDING) {
      req.user_cb(DYNAMIXEL_PACKET_ERROR, NULL);
      return;
    }
  }

  async_uart_read(rs485_buf + DYNAMIXEL_REPLY_HEADER_SIZE, body_length,
                   on_receive_reply_body);
}

static void on_packet_sent(enum async_uart_tx_err error) {
  if (error != ASYNC_UART_TX_OK) {
    req.user_cb(error, NULL);
  } else {
    async_uart_read(rs485_buf, DYNAMIXEL_REPLY_HEADER_SIZE,
                     on_receive_reply_header);
  }
}

void dynamixel_send_packet(dynamixel_request_cb callback,
                           InfoToMakeDXLPacket_t *packet) {
  assert(!packet_in_flight);
  packet_in_flight = true;

  uint16_t length = packet->generated_packet_length;
  memcpy(rs485_buf, packet->p_packet_buf, length);
  req.user_cb = callback;
  req.instr = packet->inst_idx;

  async_uart_write(rs485_buf, length, false, on_packet_sent);
}

enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet,
                                                  uint8_t *packet_buf,
                                                  dynamixel_id id) {
  DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_PING, 0,
                                     packet_buf, DYNAMIXEL_PACKET_BUF_SIZE));
  DXL_RETCHECK(end_make_dxl_packet(packet));

  return DXL_LIB_OK;
}

enum DXLLibErrorCode dynamixel_create_write_packet(
    InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, dynamixel_id id,
    uint16_t start_address, uint8_t *data, size_t data_len) {
  // https://emanual.robotis.com/docs/en/dxl/protocol2/#write-0x03
  DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_WRITE, 0,
                                     packet_buf, DYNAMIXEL_PACKET_BUF_SIZE));
  uint8_t byte1 = start_address & 0xFF;
  uint8_t byte2 = (start_address >> 8) & 0xFF;
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte1, 1));
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte2, 1));
  DXL_RETCHECK(add_param_to_dxl_packet(packet, data, data_len));
  DXL_RETCHECK(end_make_dxl_packet(packet));

  return DXL_LIB_OK;
}

enum DXLLibErrorCode dynamixel_create_read_packet(InfoToMakeDXLPacket_t *packet,
                                                  uint8_t *packet_buf,
                                                  dynamixel_id id,
                                                  uint16_t start_address,
                                                  uint16_t length) {
  // https://emanual.robotis.com/docs/en/dxl/protocol2/#read-0x02
  DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_READ, 0,
                                     packet_buf, DYNAMIXEL_PACKET_BUF_SIZE));
  uint8_t byte1 = start_address & 0xFF;
  uint8_t byte2 = (start_address >> 8) & 0xFF;
  uint8_t byte3 = length & 0xFF;
  uint8_t byte4 = (length >> 8) & 0xFF;
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte1, 1));
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte2, 1));
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte3, 1));
  DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte4, 1));
  DXL_RETCHECK(end_make_dxl_packet(packet));

  return DXL_LIB_OK;
}