#ifndef CANMORE_TITAN__REG_MAPPED_PROTOCOL_H
#define CANMORE_TITAN__REG_MAPPED_PROTOCOL_H

#include <assert.h>
#include <stdint.h>

/*
 * Register-Mapped Channel Protocol
 * ********************************
 * This defines the protocol for accessing a register-mapped interface over a CANmore utility channel.
 *
 * This allows for higher-level interface to be implemented through a variety of 'registers' accessed via addresses
 * to allow for easier implementation of higher-level protocols. Rather than defining a CAN request/response structure,
 * it can be implemented as a sequence of register accesses, which will acknowledge the recipt, and report if the access
 * was successful.
 *
 *
 * High-Level Definition
 * =====================
 *
 * All registers are 32-bits wide with a unique 16-bit address. Registers can represent a variety of objects by
 * the implementation, whether it be a single 32-bit word (to store configuration data such as a target address for an
 * operation), a range of memory (where each word represents 4 bytes of a continuous region of memory, such as a buffer
 * to store data to be accessed by an operation), or a trigger which will perform an action (to perform an action,
 * such as writing a buffer of data to a target flash address).
 *
 * Registers can be implemented as read-write, read-only, or write-only. Not all registers need to be implemented. If
 * the agent attempts to perform an invalid operation (such as writing to a read-only regsiter, or accessing an
 * unimplemented register), the response to the request will notify the agent of this error.
 *
 * All multibyte fields in the CAN frame are in little endian.
 *
 *
 * Address Structure
 * =================
 * Each register is has a unique address. To allow for simpler design, the addresses are structured in a page/offset
 * manner. Each register address constitutes a page and an offset. A page is a group of registers which share a common
 * purpose. An example could be that page 0x02 is allocated for general client status, page 0x05 for flashing, and
 * page 0x06 as a region of memory to contain data to flash. Each register in the page then has an offset to identify
 * that register in the page. For example, a register containing a unique device ID might live at offset 0x15 in
 * page 0x02, thus having a register address 0x0215. See the diagram below for how the register address is computed.
 * Note that the register address is big endian if viewed as a 2-byte field.
 *
 *   +--------+--------+
 *   | Byte 0 | Byte 1 |
 *   +--------+--------+
 *   |  Page  | Offset |
 *   +--------+--------+
 *   |   Reg Address   |
 *   +--------+--------+
 *
 * With this allocation scheme, each page can address up to 256 words, or 1024 (1K) bytes. This results in a total
 * addressable space of 64K words, or 256K bytes.
 *
 * The page-offset distinction can be ignored most of the time from the agent except for logical organization.
 * However, pages are important for multiword requests, where only certain pages support multiword requests and
 * a multiword request cannot cross a page boundary.
 *
 *
 * Bulk Requests
 * =============
 * Bulk requests are a agent to client requests which do not require a client to agent response after every request.
 * This allows for a large amount of data to be queued without the overhead of waiting for a response. This can
 * increase throughput, especially when filling a number of registers representing a buffer.
 *
 * Only write requests can be made bulk requests.
 *
 * To ensure that bulk requests do not execute unless the previous request has been successfully received, a sequence
 * number is assigned to each request. This limits the maximum number of bulk requests before a response to 256 bytes.
 * This permits 1024 bytes to be transferred before requiring a response from the client.
 *
 * Bulk requests begin with a write request with flag bulk request set to 1 and a sequence number of 0. Subsequent
 * requests can be sent with bulk request = 1 with an increasing sequence until bulk request end = 1 is set, after
 * which a response is sent. If an error occurred in any of the previous requests, a response will be sent with an
 * error result and contain the sequence number that the error occurred on. Any subsequent requests in the bulk
 * transfer were ignored.
 *
 * If a non-bulk request is sent before the request ending the bulk request, the bulk request will error bulk request
 * sequence number for the last received sequence number, and the non-bulk request will error with a bulk request
 * sequence error. Note that all non-bulk requests will fail until a bulk request with end bulk request=1 is sent.
 *
 *
 * Multiword Requests
 * ==================
 * Multiword requests allow several sequential bytes to be written at once. Only certain pages can support multiword
 * requests, and only
 *
 *
 * Agent to Client Request
 * =======================
 *
 * Read Request Structure:
 *   +--------+--------+--------+--------+
 *   | Byte 0 | Byte 1 | Byte 2 | Byte 3 |
 *   +--------+--------+--------+--------+
 *   | Flags  | Count  |  Page  | Offset |
 *   +--------+--------+--------+--------+
 *
 * Standard/Bulk Write Request Structure:
 *   +--------+--------+--------+--------+--------+--------+--------+--------+
 *   | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
 *   +--------+--------+--------+--------+--------+--------+--------+--------+
 *   | Flags  | Count  |  Page  | Offset |             Data Word             |
 *   +--------+--------+--------+--------+--------+--------+--------+--------+
 *
 * Field Definitions:
 *
 * Flags:
 *   +-*-*-*-*-+-*-+-*-+-*-+-*-+
 *   |   RFU   | M | E | B | W |
 *   +-*-*-*-*-+-*-+-*-+-*-+-*-+
 *     7     4   3   2   1   0
 *   W (Write): Set to 1 if write request, 0 if read request
 *   B (Bulk Request): Set to 1 if a bulk request, 0 if normal request
 *   E (Bulk End): Set to 1 if the last transfer in a bulk request, 0 if not last request or not a bulk request
 *   M (Multiword): Set to 1 if multiple data words are being written
 * Count: The number of words to read/write if If bulk request, an increasing counter for the bulk request
 * Reg Address: The 16-bit address for the register to access in little-endian. This is defined by the higher-level protocol
 * Data Word (Only if W=1): The 32-bit word to write in little-endian format
 *
 *
 * Client to Agent Response
 * ========================
 * A response should always be received for a request (or after a series of requests if performing a bulk request).
 * If a response is not received after a given timeout, the previous request can be re-transmitted. Care should be
 * taken by the client to ensure that repeated requests will not result in undesired results (such as if the request
 * is lost). Similarly, care should be taken by the agent to not retransmit requests while the request is still
 * being processed and responded to, as this may result in the next request receiving the previous result, causing
 * desynchronization between the request and response data.
 *
 * Write Response Structure:
 *   +--------+
 *   | Byte 0 |
 *   +--------+
 *   | Result |
 *   +--------+
 *
 * Bulk Write Response Structure:
 *   +--------+--------+
 *   | Byte 0 | Byte 1 |
 *   +--------+--------+
 *   | Result | Seq No |
 *   +--------+--------+
 *
 * Read Response Structure:
 *   +--------+--------+--------+--------+--------+
 *   | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
 *   +--------+--------+--------+--------+--------+
 *   | Result |             Data Word             |
 *   +--------+--------+--------+--------+--------+
 *
 * Result: The result from the previous requst:
 *   0: Successful
 *   1: Malformed Request
 *   2: Bulk Request Sequence Error
 *   3: Invalid Register Address
 *   4: Invalid Register Mode (Ex: Trying to write to a read-only register, multiword write across page boundary)
 * Seq No: The last sequence number received from the agent if successful, or the sequence number the error occurred on (all subsequent requests ignored)
 * Data Word: The 32-bit word read from the register in little-endian if successful, or 0 if an error occurred
 */

union reg_mapped_request_flags {
    uint8_t data;
    struct __attribute__((packed)) {
        uint8_t write:1;
        uint8_t bulk_req:1;
        uint8_t bulk_end:1;
        uint8_t multiword:1;
    } f;
};
static_assert(sizeof(union reg_mapped_request_flags) == 1, "Struct did not pack properly");

#define REG_MAPPED_MAX_REQUEST_SIZE 8

typedef union reg_mapped_request {
    uint8_t data[REG_MAPPED_MAX_REQUEST_SIZE];
    struct __attribute__((packed)) {
        union reg_mapped_request_flags flags;
        uint8_t count;
        uint8_t page;
        uint8_t offset;
        uint32_t data;
    } write_pkt;

    struct __attribute__((packed)) {
        union reg_mapped_request_flags flags;
        uint8_t count;
        uint8_t page;
        uint8_t offset;
    } read_pkt;
} reg_mapped_request_t;
static_assert(sizeof(reg_mapped_request_t) == REG_MAPPED_MAX_REQUEST_SIZE, "Struct did not pack properly");

#define REG_MAPPED_RESULT_SUCCESSFUL 0
#define REG_MAPPED_RESULT_MALFORMED_REQUEST 1
#define REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR 2
#define REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS 3
#define REG_MAPPED_RESULT_INVALID_REGISTER_MODE 4

#define REG_MAPPED_MAX_RESPONSE_SIZE 5
typedef union reg_mapped_response {
    uint8_t data[REG_MAPPED_MAX_REQUEST_SIZE];
    struct __attribute__((packed)) {
        uint8_t result;
    } write_pkt;

    struct __attribute__((packed)) {
        uint8_t result;
        uint8_t seq_no;
    } write_bulk_pkt;

    struct __attribute__((packed)) {
        uint8_t result;
        uint32_t data;
    } read_pkt;
} reg_mapped_response_t;

#endif
