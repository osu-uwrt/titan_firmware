#ifndef TITAN_BINARY_INFO__DEFS_H
#define TITAN_BINARY_INFO__DEFS_H

#include "pico/binary_info.h"

/**
 * @file titan/binary_info.h
 * @brief Contains binary info definitions for custom fields defined by the bootloader
 */

// Custom binary info tag for storing UWRT specific tags
// This allows identifying bootloader info
#define BINARY_INFO_TAG_UWRT BINARY_INFO_MAKE_TAG('U','W')
#define BINARY_INFO_ID_UW_BOOTLOADER_ENABLED 0xa4ae7ab0
#define BINARY_INFO_ID_UW_APPLICATION_BASE 0x80244d0f
#define BINARY_INFO_ID_UW_CLIENT_ID 0x8fc386f3
#define BINARY_INFO_ID_UW_DEVICE_IP_ADDRESS 0x95b9a93f
#define BINARY_INFO_ID_UW_AGENT_IP_ADDRESS 0x851b754e
#define BINARY_INFO_ID_UW_AGENT_PORT 0xb7e0a019
#define BINARY_INFO_ID_UW_VERSION_DATA 0x41370cbc

#define bi_client_id(id) bi_int(BINARY_INFO_TAG_UWRT, BINARY_INFO_ID_UW_CLIENT_ID, id)
#define bi_ip_address_array(address) (address[0] | (address[1] << 8) | (address[2] << 16) | (address[3] << 24))
#define bi_device_ip_address_array(address) bi_int(BINARY_INFO_TAG_UWRT, BINARY_INFO_ID_UW_DEVICE_IP_ADDRESS, bi_ip_address_array(address))
#define bi_agent_ip_address_array(address) bi_int(BINARY_INFO_TAG_UWRT, BINARY_INFO_ID_UW_AGENT_IP_ADDRESS, bi_ip_address_array(address))
#define bi_agent_port(port) bi_int(BINARY_INFO_TAG_UWRT, BINARY_INFO_ID_UW_AGENT_PORT, port)
#define bi_titan_version(major, minor, type) bi_int(BINARY_INFO_TAG_UWRT, BINARY_INFO_ID_UW_VERSION_DATA, \
        ((major & 0xFF) << 16) | \
        ((major & 0xFF) << 8) | \
        (type & 0xFF) \
    )

#endif