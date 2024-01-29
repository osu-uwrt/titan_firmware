#ifndef TITAN__CANMORE__SAFETY_INTERFACE_H_
#define TITAN__CANMORE__SAFETY_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Debug Interface Map
 *
 * Canmore Register Mapped Interface Definitions for debugging MCUs during normal operation (see reg_mapped_protocol.h)
 * Implemented control interface for CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL (see titan/canmore/protocol.h)
 *
 * Page Map:
 * ==========
 *
 *       +----------------+  ---- General Control Region
 * 0x00: |   MCU Control  |  (Reg)
 *       +----------------+
 * 0x01: | Version String |  RO  (Mem-mapped)
 *       +----------------+
 * 0x02: |  Memory Stats  |  (Reg)
 *       +----------------+
 * 0x03: |    GDB Stub    |  (Reg)
 *       +----------------+
 * 0x04: |   Remote Cmd   |  (Reg)
 *       +----------------+
 * 0x05: |  Rem. Cmd Args |  RW (Mem-mapped)
 *       +----------------+
 * 0x06: |  Rem. Cmd Resp |  RO (Mem-mapped)
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
 * 0x0B: |   Fault Data   |  (Reg)
 *       +----------------+
 * 0x0C: | Fault Filename |  RO  (Mem-mapped)
 *       +----------------+
 */

// ========================================
// General Control Region
// ========================================

// Page Number Definitions
#define CANMORE_DBG_MCU_CONTROL_PAGE_NUM 0x00
#define CANMORE_DBG_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_DBG_MEM_STATS_PAGE_NUM 0x02
#define CANMORE_DBG_GDB_STUB_PAGE_NUM 0x03
#define CANMORE_DBG_REMOTE_CMD_PAGE_NUM 0x04
#define CANMORE_DBG_REMOTE_CMD_ARGS_PAGE_NUM 0x05
#define CANMORE_DBG_REMOTE_CMD_RESP_PAGE_NUM 0x06
#define CANMORE_DBG_SAFETY_STATUS_PAGE_NUM 0x08
#define CANMORE_DBG_CRASH_LOG_PAGE_NUM 0x09
#define CANMORE_DBG_FAULT_NAME_PAGE_NUM 0x0A
#define CANMORE_DBG_FAULT_DATA_PAGE_NUM 0x0B
#define CANMORE_DBG_FAULT_FILENAME_PAGE_NUM 0x0C

/*
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
 */

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

/* Version String
 * ==============
 * Reading this page will return a version string. Note that the string is null terminated and attempting to read a
 * register past the 0x00 byte will result in an invalid address error.
 */

/* Memory Stats
 * ============
 * Contains information on device memory usage.
 *
 *       +----------------+
 * 0x00: |     Capture    | WO
 *       +----------------+
 * 0x01: |    Total Mem   | RO
 *       +----------------+
 * 0x02: |    Heap Use    | RO
 *       +----------------+
 * 0x03: |   Stack Use    | RO
 *       +----------------+
 * 0x04: |   Static Use   | RO
 *       +----------------+
 * 0x05: |     Arena      | RO
 *       +----------------+
 * 0x06: |    Ordblks     | RO
 *       +----------------+
 * 0x07: |     Hblks      | RO
 *       +----------------+
 * 0x08: |     Hblkhd     | RO
 *       +----------------+
 * 0x09: |    Uordblks    | RO
 *       +----------------+
 * 0x0A: |    Fordblks    | RO
 *       +----------------+
 * 0x0B: |    Keepcost    | RO
 *       +----------------+
 *
 * Capture:         Captures the current memory usage into the other registers in this page.
 *                  Must be called before reading any of the other registers in this page.
 * Total Mem:       Total memory available in the device
 * Heap Use:        Memory available to the heap
 * Stack Use:       Memory available to the core 0 stack
 * Static Use:      Memory used by static allocations
 * Arena:           Total non-mmapped bytes (mallinfo() arena)
 * Ordblks:         # of free chunks (mallinfo() ordblks)
 * Hblks:           # of mapped regions (mallinfo() hblks)
 * Hblkhd:          Bytes in mapped regions (mallinfo() hblkhd)
 * Uordblks:        Total allocated space (mallinfo() uordblks)
 * Fordblks:        Total free space (mallinfo() fordblks)
 * Keepcost:        Topmost releasable block (mallinfo() keepcost)
 */

// Memory Stats Register Definitions
#define CANMORE_DBG_MEM_STATS_CAPTURE_OFFSET 0x00
#define CANMORE_DBG_MEM_STATS_TOTAL_MEM_OFFSET 0x01
#define CANMORE_DBG_MEM_STATS_HEAP_USE_OFFSET 0x02
#define CANMORE_DBG_MEM_STATS_STACK_USE_OFFSET 0x03
#define CANMORE_DBG_MEM_STATS_STATIC_USE_OFFSET 0x04
#define CANMORE_DBG_MEM_STATS_ARENA_OFFSET 0x05
#define CANMORE_DBG_MEM_STATS_ORDBLKS_OFFSET 0x06
#define CANMORE_DBG_MEM_STATS_HBLKS_OFFSET 0x07
#define CANMORE_DBG_MEM_STATS_HBLKHD_OFFSET 0x08
#define CANMORE_DBG_MEM_STATS_UORDBLKS_OFFSET 0x09
#define CANMORE_DBG_MEM_STATS_FORDBLKS_OFFSET 0x0A
#define CANMORE_DBG_MEM_STATS_KEEPCOST_OFFSET 0x0B

/* GDB Stub
 * ========
 * Contains registers to allow debugging via GDB. All GDB requires at a minimum is a way to access device memory, and
 * know the current PC and SP.
 *
 *       +----------------+
 * 0x00: | Read Word Addr | WO
 *       +----------------+
 * 0x01: | Write Word Addr| WO
 *       +----------------+
 * 0x02: |   Memory Data  | RW
 *       +----------------+
 * 0x03: |   PC Register  | RO
 *       +----------------+
 * 0x04: |   SP Register  | RO
 *       +----------------+
 * 0x05: |   LR Register  | RO
 *       +----------------+
 *
 * Read Word Addr:  Reads the requested memory address into the memory data register. Must be in flash, ram, or rom.
 * Write Word Addr: Writes the memory data register to the requested memory address. Must be in ram.
 * Memory Data:     Register holding data to be read/written by read word addr or write word addr registers.
 * PC Register:     Reads the current PC of the debug stub, so the debugger can pull additional context from the stack
 * SP Register:     Reads the current SP of the debug stub, so the debugger can pull additional context from the stack
 * LR Register:     Reads the current SP of the debug stub, so the debugger can pull additional context from the stack
 */

// GDB Stub Register Definitions
#define CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET 0x00
#define CANMORE_DBG_GDB_STUB_WRITE_WORD_ADDR_OFFSET 0x01
#define CANMORE_DBG_GDB_STUB_MEMORY_DATA_OFFSET 0x02
#define CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET 0x03
#define CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET 0x04
#define CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET 0x05

/* Remote Cmd
 * ==========
 * Allows executing commands in the standard "command arg1 arg2" format, and receiving back output. Implementing new
 * canmore CLI commands via this register interface requires a lot of work (as this requires editing this file, the
 * debug server in firmware, the debug client in canmore cli, then adding a command handler to Canmore CLI). I should
 * know since I need to do all of that to implement this remote command interface.
 *
 * However, after adding this interface, all that is required to create a new command is make a simple string based
 * stub in the firmware, which handles the standard argc argv format used by posix, then filling out the result string
 * to be displayed by canmore CLI.
 *
 * This should make it significantly easier to register new commands for debugging, which should allow canmore CLI to be
 * even more valuable when testing prototype code.
 *
 *       +----------------+
 * 0x00: |    Execute     | RO
 *       +----------------+
 *
 * Execute: Reading this register executes the command contained in the Remote Command Args page. The result string is
 *          written to the remote command response page. Additionally, the return code is read out of this regsiter
 *          after execution.
 */

// Remote Cmd Register Definitions
#define CANMORE_DBG_REMOTE_CMD_EXECUTE_OFFSET 0x00

/* Remote Cmd Args
 * ===============
 * Write the command and arguments to execute as a remote command to this register page.
 *
 * The command is written, followed by a null character. Each argument is then written, separated by null characters.
 * After the last argument, a null-terminated empty string is placed, signifying the end of arguments. All other data
 * after the empty string ignored.
 *
 * If the command "test arg1 arg2 arg3" was to be executed, resulting in the command "test" being ran with arguments
 * {"arg1", "arg2", "arg3"}, the data would be encoded as:
 *
 *     "test\0arg1\0arg2\0arg3\0\0"
 *
 * This register is set to "\0" to avoid multiple reads from accidentally executing the command multiple times if the
 * execute register is read twice.
 * Note that all bytes after the first is still preserved (as the command decoder stops after the first empty string).
 */

#define CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN REG_MAPPED_PAGE_SIZE

/* Remote Cmd Resp
 * ===============
 *
 * Page containing the response from the remote command as a string. This will ALWAYS be updated after each execute,
 * even if the command does not exist. If this is the case, the command handler is responsible for updating this string.
 */

#define CANMORE_DBG_REMOTE_CMD_RESP_MAX_LEN REG_MAPPED_PAGE_SIZE

// ========================================
// Safety Status Region
// ========================================

/* Safety Status Register Map
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
 * 0x03: |    Fault Idx   | RW
 *       +----------------+
 * 0x04: |   Raise Fault  | WO
 *       +----------------+
 * 0x05: |   Lower Fault  | WO
 *       +----------------+
 *
 * Global State:    Contains bit flags reporting the global safety state
 *      Bit 0:        Safety Setup
 *      Bit 1:        Safety Initialized
 *      Bit 2:        Safety Kill State is Enabled
 *      Bit 3:        Safety Fault Present
 * Fault List:      Bitwise list of all faults present
 * Uptime:          System uptime in centiseconds
 * Fault Idx:       Index for fault in fault name, data, and filename page
 * Raise Fault:     Raises the requested fault id
 * Lower Fault:     Lowers the requested fault id
 */

// Safety Status Register Map
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET 0x00
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_SETUP_FIELD 0
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_INITIALIZED_FIELD 1
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_KILL_IS_ENABLED_FIELD 2
#define CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_FAULT_PRESENT_FIELD 3
#define CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET 0x01
#define CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET 0x02
#define CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND 100
#define CANMORE_DBG_SAFETY_STATUS_FAULT_IDX_OFFSET 0x03
#define CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET 0x04
#define CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET 0x05

/* Crash Log
 * =========
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
 * Prev Scratch 1:  The scratch 1 register for the previous reset (or reason for clean boot if reset reason is clean
 * boot). Not valid until Prev Index is written Prev Scratch 2:  The scratch 2 register for the previous reset. Not
 * valid until Prev Index is written
 */

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
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_ASSERT_VALUE 7
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_WATCHDOG_TIMEOUT_VALUE 8
#define CANMORE_DBG_CRASH_LOG_RESET_REASON_CORE1_TIMEOUT_VALUE 9
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

/* Fault Name
 * ==========
 * Contains string of the fault name specified in the Fault Idx Register. Note until the Fault Idx register is written
 * this page cannot be read.
 */

/* Fault Data
 * ==========
 * Contains data for the fault selected by the Fault Idx Register. Note until the Fault Idx register is written this
 * page cannot be read.
 *
 *       +----------------+
 * 0x00: |     Sticky     | RO
 *       +----------------+
 * 0x01: |   Time Lower   | RO
 *       +----------------+
 * 0x02: |   Time Upper   | RO
 *       +----------------+
 * 0x03: |   Extra Data   | RW
 *       +----------------+
 * 0x04: |  Line Number   | RO
 *       +----------------+
 * 0x05: | Multiple Fires | RO
 *       +----------------+
 *
 * Sticky:          True if the fault has ever been raised. All other fault data is invalid if this is false
 * Time Lower:      The lower 32-bits of the timestamp when the fault was raised (in us)
 * Time Upper:      The upper 32-bits of the timestamp when the fault was raised (in us)
 * Extra Data:      Optional extra data that was passed with the fault being raised
 * Line Number:     The line number where the fault was raised
 * Multiple Fires:  True if the fault was raised multiple times (data was overwritten)
 *
 *
 * Fault Filename
 * ==============
 * Contains the filename that the last fault was raised. This page cannot be read until the Fault Idx register is
 * written. Additionally this will be invalid unless Fault Data::Sticky is true.
 *
 */

// Fault Data Page
#define CANMORE_DBG_FAULT_DATA_STICKY_OFFSET 0x00
#define CANMORE_DBG_FAULT_DATA_TIME_LOWER_OFFSET 0x01
#define CANMORE_DBG_FAULT_DATA_TIME_UPPER_OFFSET 0x02
#define CANMORE_DBG_FAULT_DATA_EXTRA_DATA_OFFSET 0x03
#define CANMORE_DBG_FAULT_DATA_LINE_NUMBER_OFFSET 0x04
#define CANMORE_DBG_FAULT_DATA_MULTIPLE_FIRES_OFFSET 0x05

#ifdef __cplusplus
}
#endif

#endif
