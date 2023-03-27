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
 * 0x01: | Version String |  RO  (Mem-mapped)
 *       +----------------+
 *       |                |
 *       |      ...       |
 *       |                |
 *       +----------------+  ---- Flash Control Region
 * 0x08: | Flash Control  |  (Reg)
 *       +----------------+
 * 0x09: |  Flash Buffer  |  RW  (Mem-mapped)
 *       +----------------+
 *
 *
 * MCU Control Register Map
 * ========================
 * Contains registers related to core MCU control and identification
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
 *
 *
 * Flash Control Register Map
 * ==========================
 * Controls registers related to controlling the flash
 *
 *       +----------------+
 * 0x00: |    Command     | WO
 *       +----------------+
 * 0x01: |  Target Addr   | RW
 *       +----------------+
 * 0x02: |  BL Write Key  | RW
 *       +----------------+
 * 0x03: |      CRC       | RO
 *       +----------------+
 * 0x04: |   Flash Size   | RO
 *       +----------------+
 *
 * Command:         The command to execute. If the command or target addr is invalid (invalid region or unaligned), the
 *                  register write will error with bad data. The register write will not respond until the operation has
 *                  finished
 *                      - READ: Reads 256 bytes of target addr into Flash Buffer. Address be 256-byte aligned
 *                      - WRITE: Writes 256 bytes of Flash Buffer to the target address. Address be 256-byte aligned
 *                      - ERASE: Erases the 4KB page specified by target address. Address must be 4KB-aligned
 *                      - CRC: Computes a CRC-32 polynomial for the contents in Flash Buffer
 * Target Addr:     The target address for the command
 * BL Write Key:    Register to allow erasing/writing the bootloader region. If this contains the bootloader write value,
 *                  the bootloader can be erased/written with the command. If this contains any other value, attepting to
 *                  erase/program this region will error with invalid address.
 * CRC:             Holds the result CRC calculated with the CRC command
 * Flash Size:      The flash size in bytes
 *
 *
 * Flash Buffer
 * ============
 * This page is a 256-byte read/write buffer to hold contents of the flash commands specified above.
 *
*/

// Page Number Definitions
#define CANMORE_BL_MCU_CONTROL_PAGE_NUM 0x00
#define CANMORE_BL_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_BL_FLASH_CONTROL_PAGE_NUM 0x08
#define CANMORE_BL_FLASH_BUFFER_PAGE_NUM 0x09
#define CANMORE_BL_FLASH_BUFFER_SIZE 256

// MCU Control Register Definitions
#define CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET 0x00
#define CANMORE_BL_MCU_CONTROL_MAGIC_VALUE 0x10ad2040
#define CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET 0x01
#define CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET 0x02
#define CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET 0x03
#define CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID 0x04
#define CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID 0x05
#define CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET 0x06

// Flash Control Register Map
#define CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET 0x00
#define CANMORE_BL_FLASH_CONTROL_COMMAND_READ 0x00
#define CANMORE_BL_FLASH_CONTROL_COMMAND_WRITE 0x01
#define CANMORE_BL_FLASH_CONTROL_COMMAND_ERASE 0x02
#define CANMORE_BL_FLASH_CONTROL_COMMAND_CRC 0x03
#define CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET 0x01
#define CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET 0x02
#define CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_VALUE 0x55AA55AA
#define CANMORE_BL_FLASH_CONTROL_CRC_OFFSET 0x03
#define CANMORE_BL_FLASH_CONTROL_FLASH_SIZE_OFFSET 0x04

// Target Address Alignment
#define CANMORE_BL_FLASH_ERASE_SIZE 0x1000
#define CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK 0xFF
#define CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK 0xFF
#define CANMORE_BL_FLASH_ERASE_ADDR_ALIGN_MASK 0xFFF

#endif