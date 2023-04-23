#ifndef BL_BINARY_INFO__DEFS_H
#define BL_BINARY_INFO__DEFS_H

/**
 * @file bl_binary_info/defs.h
 * @brief Contains binary info definitions for custom fields defined by the bootloader
 */

// Custom binary info tag for storing UWRT specific tags
// This allows identifying bootloader info
#define BINARY_INFO_TAG_UWRT BINARY_INFO_MAKE_TAG('U','W')
#define BINARY_INFO_ID_UW_BOOTLOADER_ENABLED 0xa4ae7ab0
#define BINARY_INFO_ID_UW_APPLICATION_BASE 0x80244d0f

#endif