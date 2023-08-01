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
 * 0x09: |   Crash Log    |  (Reg)
 *       +----------------+
 * 0x0A: |   Fault Name   |  RO  (Mem-mapped)
 *       +----------------+
 * 0x0B: | Profiler Data  |  RO  (Mem-mapped)
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
 * 0x00: |  Global State  | RO
 *       +----------------+
 * 0x01: |   Fault List   | RO
 *       +----------------+
 * 0x02: |     Uptime     | RO
 *       +----------------+
 * 0x03: | Fault Name Idx | RW
 *       +----------------+
 * 0x04: |   Raise Fault  | WO
 *       +----------------+
 * 0x05: |   Lower Fault  | WO
 *       +----------------+
 *
 *
 * Global State:    Contains bit flags reporting the global safety state
 *      Bit 0:        Safety Setup
 *      Bit 1:        Safety Initialized
 *      Bit 2:        Safety Kill State is Enabled
 *      Bit 3:        Safety Fault Present
 * Fault List:      Bitwise list of all faults present
 * Uptime:          System uptime in centiseconds
 * Fault Name Idx:  Index for fault name to be present in Fault Name Page
 * Raise Fault:     Raises the requested fault id
 * Lower Fault:     Lowers the requested fault id
 *
 * Crash Log
 * ==============
 * Contains registers to read through the MCU crash log
 *
 *       +----------------+
 * 0x00: |   Crash Count  | RO
 *       +----------------+
 * 0x01: |   Has Wrapped  | RO
 *       +----------------+
 * 0x02: |   Prev Count   | RO
 *       +----------------+
 * 0x03: |   Prev Index   | RW
 *       +----------------+
 * 0x04: |  Reset Reason  | RO
 *       +----------------+
 * 0x05: |Prev Fault List | RO
 *       +----------------+
 * 0x06: |   Prev Uptime  | RO
 *       +----------------+
 * 0x07: | Prev Scratch 1 | RO
 *       +----------------+
 * 0x08: | Prev Scratch 2 | RO
 *       +----------------+
 *
 * Crash Count:     The crash counter since the last clean boot
 *      Bits 0-7:     Total crash count
 *      Bits 8-15:    Panic Count
 *      Bits 16-23:   Hard Fault Count
 *      Bits 24-31:   Assertion Fail Count
 * Has Wrapped:     Set to 1 if the crash log has wrapped around (and lost entries), 0 if not
 * Prev Entries:    Contains count of previous entries stored in this crash log
 * Prev Index:      The index to be read from the crash log. This updates the Reset Reason -> Prev Scratch 2 Regsisters
 * Reset Reason:    The reason for the previous reset. Not valid until Prev Index is written
 * Prev Fault List: The fault list from the previous reset. Not valid until Prev Index is written
 * Prev Uptime:     The uptime from the previous reset. Not valid until Prev Index is written
 * Prev Scratch 1:  The scratch 1 register for the previous reset (or reason for clean boot if reset reason is clean boot).
 *                  Not valid until Prev Index is written
 * Prev Scratch 2:  The scratch 2 register for the previous reset. Not valid until Prev Index is written
 *
 * Fault Name
 * ==============
 * Contains string of the fault name specified in the Fault Name Idx Register. Note until the Fault Name Idx register
 * is written this page cannot be read.
 *
 */

// Page Number Definitions
#define CANMORE_DBG_MCU_CONTROL_PAGE_NUM 0x00
#define CANMORE_DBG_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_DBG_SAFETY_STATUS_PAGE_NUM 0x08
#define CANMORE_DBG_CRASH_LOG_PAGE_NUM 0x09
#define CANMORE_DBG_FAULT_NAME_PAGE_NUM 0x0A
#define CANMORE_DBG_PROFILER_PAGE_NUM 0x0B  // TODO Add in

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
#define CANMORE_DBG_MCU_CONTROL_CAN_INTR_EN_OFFSET 0x08  // TODO: Remove me when bug fixed
#define CANMORE_DBG_MCU_CONTROL_CAN_FIFO_CLEAR_OFFSET 0x09
#define CANMORE_DBG_MCU_CONTROL_CAN_RESET_OFFSET 0x0A

// Safety Status Register Map
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET 0x00
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_SETUP_FIELD 0
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_INITIALIZED_FIELD 1
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_KILL_IS_ENABLED_FIELD 2
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_FAULT_PRESENT_FIELD 3
#define CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET 0x01
#define CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET 0x02
#define CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND 100
#define CANMORE_DBG_SAFETY_STATUS_FAULT_NAME_IDX_OFFSET 0x03
#define CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET 0x04
#define CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET 0x05

// Crash Log Register Map
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_OFFSET 0x00
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_SHIFT 0
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_MASK 0xFF
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_SHIFT 8
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_MASK 0xFF
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_SHIFT 16
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_MASK 0xFF
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_SHIFT 24
#define CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_MASK 0xFF
#define CANMORE_DBG_CRASH_LOG_HAS_WRAPPED_OFFSET 0x01
#define CANMORE_DBG_CRASH_LOG_PREV_COUNT_OFFSET 0x02
#define CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET 0x03
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET 0x04
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_CLEAN_BOOT_VALUE 0
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_PREINIT_VALUE 1
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_ACTIVE_VALUE 2
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_PANIC_VALUE 3
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_FAULT_VALUE 4
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_ASSERT_FAIL_VALUE 5
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_TIMEOUT_DURING_ROS_VALUE 6
#define CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET 0x05
#define CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET 0x06
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET 0x07
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_POR_VALUE 1
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_RUN_VALUE 2
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_PSM_VALUE 3
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_SOFTWARE_VALUE 4
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_UNK_WDG_VALUE 5
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_BOOTLOADER_VALUE 6
#define CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET 0x08


#ifdef __cplusplus
}
#endif

#endif
