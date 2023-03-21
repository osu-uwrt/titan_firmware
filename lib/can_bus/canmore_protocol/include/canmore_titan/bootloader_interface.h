#ifndef CANMORE_TITAN__BOOTLOADER_INTERFACE_H
#define CANMORE_TITAN__BOOTLOADER_INTERFACE_H

/*
 * Bootloader Interface Map
 *
 * Page Map:
 * ==========
 *
 *       +----------------+  ---- General Control Region
 * 0x00: |   MCU Control  |  (Reg)
 *       +----------------+
 * 0x01: | Version String |  (Mem-mapped)
 *       +----------------+
 *       |                |
 *       |      ...       |
 *       |                |
 *       +----------------+  ---- Flash Control Region
 * 0x08: | Flash Control  |  (Reg)
 *       +----------------+
 * 0x09: |  Write Buffer  |  (Mem-mapped)
 *       +----------------+
 * 0x0A: |  Flash Mirror  |  (Mem-mapped)
 *       +----------------+
 *
 *
 * MCU Control Register Map
 * ========================
 * Contains registers related to the
 *
 *       +----------------+
 * 0x00: |     Magic      | RO
 *       +----------------+
 * 0x01: | Major Version  | RO
 *       +----------------+
 * 0x02: | Minor Version  | RO
 *       +----------------+
 * 0x03: |  Release Type  | RO
 *       +----------------+
 * 0x04: | Lower Flash ID | RO
 *       +----------------+
 * 0x05: | Upper Flash ID | RO
 *       +----------------+
 * 0x06: |  Reboot MCU    | WO
 *       +----------------+
 *
 *
 * Magic:           Magic value to identify valid control block
 *                  Should contain the word 0x10ad2040
 * Major Version:   Contains the major version of the bootloader
 *                  Can be used to identify incompatible versions
 * Minor Version:   Contains the minor version of the bootloader
 *                  Can be used to identify incompatible versions
 * Release Type:    Contains the release type of the bootloader
 * Lower Flash ID:  The unique ID lower bytes of the flash connected to the RP2040
 * Upper Flash ID:  The unique ID upper bytes of the flash connected to the RP2040
 * Reboot MCU:      Writing a value to this register will reboot the microcontroller
 *
 *
 * Version String
 * ==============
 * Reading this page will return a version string. Note that the string is null terminated and attempting to read a
 * register past the 0x00 byte will result in an invalid address error.
*/

// Page Number Definitions
#define CANMORE_BL_MCU_CONTROL_PAGE_NUM 0x00
#define CANMORE_BL_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_BL_FLASH_CONTROL_PAGE_NUM 0x08
#define CANMORE_BL_WRITE_BUFFER_PAGE_NUM 0x09
#define CANMORE_BL_FLASH_MIRROR_PAGE_NUM 0x0A

// MCU Control Register Definitions
#define CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET 0x00
#define CANMORE_BL_MCU_CONTROL_MAGIC_VALUE 0x10ad2040
#define CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET 0x01
#define CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET 0x02
#define CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET 0x03
#define CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID 0x04
#define CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID 0x05
#define CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET 0x06



#endif