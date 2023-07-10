#ifndef TITAN__CANMORE__SAFETY_INTERFACE_H_
#define TITAN__CANMORE__SAFETY_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Debug Interface Map
 * Implemented control interface for CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL
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
 *       +----------------+  ---- Safety Status Region
 * 0x08: | Safety Status  |  (Reg)
 *       +----------------+
 * 0x09: |   Crash Log    |  RO  (Mem-mapped)
 *       +----------------+
 * 0x0A: | Profiler Data  |  RO  (Mem-mapped)
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
 * 0x01: |    Enter BL    | WO
 *       +----------------+
 * 0x02: | Lower Flash ID | RO
 *       +----------------+
 * 0x03: | Upper Flash ID | RO
 *       +----------------+
 * 0x04: | Major Version  | RO
 *       +----------------+
 * 0x05: | Minor Version  | RO
 *       +----------------+
 * 0x06: |  Release Type  | RO
 *       +----------------+
 * 0x07: |   Reboot MCU   | WO
 *       +----------------+
 *
 *
 * Magic:           Magic value to identify valid control block
 *                  Should contain the word 0x10ad2040
 * Enter BL:        Writing a value to this register will reboot the microcontroller into bootloader mode
 * Lower Flash ID:  The unique ID lower bytes of the flash connected to the RP2040
 * Upper Flash ID:  The unique ID upper bytes of the flash connected to the RP2040
 * Major Version:   Contains the major version of the bootloader
 *                  Can be used to identify incompatible versions
 * Minor Version:   Contains the minor version of the bootloader
 *                  Can be used to identify incompatible versions
 * Release Type:    Contains the release type of the bootloader
 * Reboot MCU:      Writing a value to this register will reboot the microcontroller
 *
 *
 * Version String
 * ==============
 * Reading this page will return a version string. Note that the string is null terminated and attempting to read a
 * register past the 0x00 byte will result in an invalid address error.
 *
 *
 * Safety Status Register Map
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
 * 0x05: |    App Base    | RO
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
 * App Base:        The flash address where the application image is expected by the bootloader
 *
 *
 * Flash Buffer
 * ============
 * This page is a 256-byte read/write buffer to hold contents of the flash commands specified above.
 *
*/

// Page Number Definitions
#define CANMORE_DBG_MCU_CONTROL_PAGE_NUM 0x00
#define CANMORE_DBG_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_DBG_SAFETY_STATUS_PAGE_NUM 0x08
#define CANMORE_DBG_CRASH_LOG_PAGE_NUM 0x09
#define CANMORE_DBG_PROFILER_PAGE_NUM 0x0A

// MCU Control Register Definitions
#define CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET 0x00
#define CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE 0x0DBAA1F0
#define CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET 0x01
#define CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID 0x02
#define CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID 0x03
#define CANMORE_DBG_MCU_CONTROL_MAJOR_VERSION_OFFSET 0x04
#define CANMORE_DBG_MCU_CONTROL_MINOR_VERSION_OFFSET 0x05
#define CANMORE_DBG_MCU_CONTROL_RELEASE_TYPE_OFFSET 0x06
#define CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET 0x07

// Safety Status Register Map
// TODO Add in

#ifdef __cplusplus
}
#endif

#endif