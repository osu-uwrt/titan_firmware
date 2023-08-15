#include "dynamixel_comms.h"

#include "async_uart.h"
#include "dxl_packet.h"

#include "driver/dynamixel.h"

#include <assert.h>
#include <string.h>

#define PROTOCOL 2  // This file supports protocol v2
/** Includes all of the bytes up to the packet length. */
#define DYNAMIXEL_REPLY_HEADER_SIZE 8
#define DXL_RETCHECK(fn)                                                                                               \
    {                                                                                                                  \
        enum DXLLibErrorCode temp_rc = fn;                                                                             \
        if ((temp_rc != DXL_LIB_OK)) {                                                                                 \
            return temp_rc;                                                                                            \
        }                                                                                                              \
    }

#define dynamixel_comms_report_error_with_arg(error_code, arg)                                                         \
    do {                                                                                                               \
        dynamixel_error_t error_msg = { .fields = { .error = error_code,                                               \
                                                    .error_source = DYNAMIXEL_SOURCE_COMMS,                            \
                                                    .line = __LINE__,                                                  \
                                                    .wrapped_error_code = arg } };                                     \
        req.user_cb(error_msg, NULL);                                                                                  \
    } while (0)

#define dynamixel_comms_report_error(error_code) dynamixel_comms_report_error_with_arg(error_code, 0)

#define check_transfer_dxl_call(ret_code, expected)                                                                    \
    do {                                                                                                               \
        enum DXLLibErrorCode ret = ret_code;                                                                           \
        if (ret != expected) {                                                                                         \
            packet_in_flight = false;                                                                                  \
            dynamixel_comms_report_error_with_arg(DYNAMIXEL_PACKET_ERROR, ret);                                        \
            return;                                                                                                    \
        }                                                                                                              \
    } while (0)

// ========================================
// Packet Transmission
// ========================================

struct current_request {
    dynamixel_request_cb user_cb;
    enum DXLInstruction instr;
    uint8_t request_id;
};

static volatile bool packet_in_flight = false;
static InfoToParseDXLPacket_t rx_packet;
static uint8_t rx_packet_param_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];

/** Stores the current internal request that is happening. */
static struct current_request req;
/** Used internally as the read/write buffer during transfer. */
static uint8_t uart_header_buf[DYNAMIXEL_REPLY_HEADER_SIZE];
static uint8_t uart_buf[DYNAMIXEL_PACKET_BUFFER_SIZE + 3];  // Add additional 3 bytes for CRC and error fields

static void on_receive_reply_body(enum async_uart_rx_err error, uint8_t *data, size_t data_len) {
    assert(packet_in_flight);

    if (error != ASYNC_UART_RX_OK) {
        packet_in_flight = false;
        dynamixel_comms_report_error_with_arg(DYNAMIXEL_TTL_ERROR, error);
        return;
    }

    // Begin parsing the packet
    check_transfer_dxl_call(
        begin_parse_dxl_packet(&rx_packet, PROTOCOL, rx_packet_param_buf, sizeof(rx_packet_param_buf)), DXL_LIB_OK);

    // give the header data to the parser.
    for (size_t i = 0; i < DYNAMIXEL_REPLY_HEADER_SIZE; i++) {
        check_transfer_dxl_call(parse_dxl_packet(&rx_packet, uart_header_buf[i]), DXL_LIB_PROCEEDING);
    }

    // Parse the body
    for (size_t i = 0; i < data_len - 1; i++) {
        check_transfer_dxl_call(parse_dxl_packet(&rx_packet, data[i]), DXL_LIB_PROCEEDING);
    }

    // Ensure the last byte finishes the parse
    check_transfer_dxl_call(parse_dxl_packet(&rx_packet, data[data_len - 1]), DXL_LIB_OK);

    // Report result to the user callback
    struct dynamixel_req_result result;
    result.instr = req.instr;
    result.request_id = req.request_id;
    result.packet = &rx_packet;
    packet_in_flight = false;

    dynamixel_error_t errcode = {
        .fields = { .error = DYNAMIXEL_ERROR_NONE, .line = __LINE__, .error_source = DYNAMIXEL_SOURCE_COMMS }
    };
    req.user_cb(errcode, &result);
}

static void on_receive_reply_header(enum async_uart_rx_err error, uint8_t *data, size_t data_len) {
    assert(packet_in_flight);

    if (error != ASYNC_UART_RX_OK) {
        packet_in_flight = false;
        dynamixel_comms_report_error_with_arg(DYNAMIXEL_TTL_ERROR, error);
        return;
    }

    // We asked for DYNAMIXEL_REPLY_HEADER_SIZE, the driver should have given us this many bytes or we woulnd't get OK
    (void) data_len;
    assert(data_len == DYNAMIXEL_REPLY_HEADER_SIZE);

    uint16_t body_length = data[5] | (data[6] << 8);

    // Ensure that the reported packet length isn't too big for our buffer
    if (body_length < 2 || body_length > sizeof(uart_buf) + 1) {
        packet_in_flight = false;
        dynamixel_comms_report_error(DYNAMIXEL_PACKET_ERROR);
        return;
    }

    // Immediately begin turnaround and send body
    // The length includes the last byte of the header (instruction byte)
    async_uart_read(uart_buf, body_length - 1, on_receive_reply_body);
}

static void on_packet_sent(enum async_uart_tx_err error) {
    assert(packet_in_flight);

    if (error != ASYNC_UART_TX_OK) {
        packet_in_flight = false;
        dynamixel_comms_report_error_with_arg(DYNAMIXEL_TTL_ERROR, error);
        return;
    }

    async_uart_read(uart_header_buf, DYNAMIXEL_REPLY_HEADER_SIZE, on_receive_reply_header);
}

void dynamixel_send_packet(dynamixel_request_cb callback, InfoToMakeDXLPacket_t *packet) {
    assert(!packet_in_flight);

    uint16_t length = packet->generated_packet_length;
    if (length > sizeof(uart_buf)) {
        dynamixel_comms_report_error(DYNAMIXEL_REQUEST_ERROR);
        return;
    }

    packet_in_flight = true;
    memcpy(uart_buf, packet->p_packet_buf, length);
    req.user_cb = callback;
    req.instr = packet->inst_idx;
    req.request_id = packet->id;

    async_uart_write(uart_buf, length, false, on_packet_sent);
}

// ========================================
// Packet Generation
// ========================================

enum DXLLibErrorCode dynamixel_create_ping_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id) {
    DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_PING, 0, packet_buf, packet_buf_size));
    DXL_RETCHECK(end_make_dxl_packet(packet));

    return DXL_LIB_OK;
}

enum DXLLibErrorCode dynamixel_create_write_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                   size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                   uint8_t *data, size_t data_len) {
    // https://emanual.robotis.com/docs/en/dxl/protocol2/#write-0x03
    DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_WRITE, 0, packet_buf, packet_buf_size));
    uint8_t byte1 = start_address & 0xFF;
    uint8_t byte2 = (start_address >> 8) & 0xFF;
    DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte1, 1));
    DXL_RETCHECK(add_param_to_dxl_packet(packet, &byte2, 1));
    DXL_RETCHECK(add_param_to_dxl_packet(packet, data, data_len));
    DXL_RETCHECK(end_make_dxl_packet(packet));

    return DXL_LIB_OK;
}

enum DXLLibErrorCode dynamixel_create_read_packet(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf,
                                                  size_t packet_buf_size, dynamixel_id id, uint16_t start_address,
                                                  uint16_t length) {
    // https://emanual.robotis.com/docs/en/dxl/protocol2/#read-0x02
    DXL_RETCHECK(begin_make_dxl_packet(packet, id, PROTOCOL, DXL_INST_READ, 0, packet_buf, packet_buf_size));
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
