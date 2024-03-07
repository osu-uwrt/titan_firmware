#ifndef TITAN__CANMORE__LINUX_INTERFACE_H_
#define TITAN__CANMORE__LINUX_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Linux Interface Map
 *
 * Canmore Register Mapped Interface Definitions for interfacing with a Linux machine over CAN bus (e.g. the downwards
 * facing camera). This is on the fringes of what CANmore was designed for (it's more for microcontrollers), but all of
 * this infrastructure and tooling has been built to allow reliable communication over the CAN bus link, so we might as
 * well use it.
 *
 * Page Map:
 * ==========
 *
 *       +----------------+  ---- General Control Region
 * 0x00: |  Gen. Control  |  (Reg)
 *       +----------------+
 * 0x01: | Version String |  RO  (Mem-mapped)
 *       +----------------+
 * 0x02: |   TTY Control  |  (Reg)
 *       +----------------+
 * 0x03: |  TTY Terminal  |  RW  (Mem-mapped)
 *       +----------------+
 * 0x04: |     Command    |  RW  (Mem-mapped)
 *       +----------------+
 * 0x05: |   File Buffer  |  RW  (Mem-mapped)
 *       +----------------+
 * 0x06: | Upload Control |  (Reg)
 *       +----------------+
 */

// Page Number Definitions
#define CANMORE_LINUX_GEN_CONTROL_PAGE_NUM 0x00
#define CANMORE_LINUX_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_LINUX_TTY_CONTROL_PAGE_NUM 0x02
#define CANMORE_LINUX_TTY_TERMINAL_PAGE_NUM 0x03
#define CANMORE_LINUX_TTY_CMD_PAGE_NUM 0x04
#define CANMORE_LINUX_FILE_BUFFER_PAGE_NUM 0x05
#define CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM 0x06

/* Linux Control Register Map
 * ==========================
 * Contains registers related to core linux daemon control and identification
 *
 *       +----------------+
 * 0x00: |     Magic      | RO
 *       +----------------+
 * 0x01: | Lower Flash ID | RO
 *       +----------------+
 * 0x02: | Upper Flash ID | RO
 *       +----------------+
 * 0x03: |  Reboot Device | WO
 *       +----------------+
 * 0x04: | Restart Daemon | WO
 *       +----------------+
 *
 *
 * Magic:           Magic value to identify valid control block
 *                  Should contain the word 0x4E6D2B07
 * Lower Flash ID:  The unique ID lower bytes of the machine
 * Upper Flash ID:  The unique ID upper bytes of the machine
 * Reboot Device:   Writing 0xFEE1DEAD to this register will reboot the machine
 * Restart Daemon:  Writing 0xDEADFA11 to this resgister restarts the debug server
 */

// MCU Control Register Definitions
#define CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET 0x00
#define CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE 0x4E6D2B07
#define CANMORE_LINUX_GEN_CONTROL_LOWER_FLASH_ID 0x01
#define CANMORE_LINUX_GEN_CONTROL_UPPER_FLASH_ID 0x02
#define CANMORE_LINUX_GEN_CONTROL_REBOOT_DEVICE_OFFSET 0x03
#define CANMORE_LINUX_GEN_CONTROL_REBOOT_DEVICE_MAGIC 0xFEE1DEAD
#define CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_OFFSET 0x04
#define CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_MAGIC 0xDEADFA11

/* Version String
 * ==============
 * Reading this page will return a version string. Note that the string is null terminated and attempting to read a
 * register past the 0x00 byte will result in an invalid address error.
 */

/* TTY Control
 * ===========
 * Contains registers to control the debug TTY interface exposed over CAN bus (exposed on other utlity channel).
 *
 *       +----------------+
 * 0x00: |     Enable     | RW
 *       +----------------+
 * 0x01: |  Window Size   | RW
 *       +----------------+
 *
 * Enable:          Writing 1 to this register will initialize the remote tty session. Writing 0 will kill the session.
 * Window Size:     Sets the TTY's initial window size. Lower 16-bits are num cols, upper 16-bits are num rows.
 *
 */

// TTY Control Definitions
#define CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET 0x00
#define CANMORE_LINUX_TTY_CONTROL_WINDOW_SIZE_OFFSET 0x01

/* TTY Terminal
 * ============
 * Writing this page will set the TERM environment variable used when initializing the TTY. Note that this must be set
 * before writing Initialize to 1.
 *
 * This should be written as a normal NULL terminated string
 */

/* TTY Command
 * ============
 * This page controls the initial command to run. If this page contains an empty string, the default login shell is ran.
 * Note that this must be set before writing Initialize to 1.
 *
 * This should be written as a normal NULL terminated string
 */

/**
 * File Buffer
 * ===========
 * This page serves as a buffer in which to store file data before a file write or after a file read is performed on the
 * remote device by using the registers in the Upload Control or Download Control pages.
 *
 * When using the buffer to perform a WRITE operation on a file, the contents of the registers must be as follows:
 *
 * register 0-n-1: Name of the file to write.
 * register n-x:   Data to write to the file.
 *
 * Where: n is the minimal number of registers (4 bytes / register) required to express the filename, and x is the
 * number of additional registers needed to store the data to write.
 *
 * When a write is triggered using the Upload Control page, the file data
 * will, unless otherwise specified, be appended to the file specified by filename. After a write, the buffer will
 * automatically be cleared.
 */

/**
 * Upload Control
 * ==============
 * Contains registers to control the data stored in the Upload Buffer page.
 *
 *       +-----------------+
 * 0x00: | Filename Length |  WO
 *       +-----------------+
 * 0x01  |   Data Length   | RW
 *       +-----------------+
 * 0x01: |      CRC32      | WO
 *       +-----------------+
 * 0x02: |   Clear File    | WO
 *       +-----------------+
 * 0x03: |    File Mode    | WO
 *       +-----------------+
 * 0x04: |      Write      | WO
 *       +-----------------+
 * 0x05: |   Write Status  | RO
 *       +-----------------+
 *
 * Filename Length: The length of the filename in bytes. The data should start at the beginning of the next register.
 *
 * Data Length:     The number of bytes (not regs) to write from the buffer into the file.
 *
 * CRC32:           The 32-bit CRC of the data currently present in the File Buffer page. If this does not match the
 *                  CRC32 calculated on the remote device when the write action is triggered, the write will fail and
 *                  the Write Status register will be set accordingly.
 *
 * Clear File:      If set to 0 (false), a write action will append the file data contents of the File Buffer page to
 *                  the end of the file. Otherwise, a write action will delete the file data contents of the file on the
 *                  remote device before writing the contents of the File Buffer page into it
 *
 * File Mode:       The desired mode of the file.
 *
 * Write:           Writing 1 to this register will initiate a write action. Before triggering another write action, 0
 *                  must be written to the register to clear any errors and set the write status to ready.
 *
 * Write Status:    Contains the current status of the remote device filewriter. Possible values are described below
 *                  where the status macros are defined.
 */

#define CANMORE_LINUX_UPLOAD_CONTROL_FILENAME_LENGTH_OFFSET 0x00
#define CANMORE_LINUX_UPLOAD_CONTROL_DATA_LENGTH_OFFSET 0x01
#define CANMORE_LINUX_UPLOAD_CONTROL_CRC_OFFSET 0x02
#define CANMORE_LINUX_UPLOAD_CONTROL_CLEAR_FILE_OFFSET 0x03
#define CANMORE_LINUX_UPLOAD_CONTROL_FILE_MODE_OFFSET 0x04
#define CANMORE_LINUX_UPLOAD_CONTROL_WRITE_OFFSET 0x05
#define CANMORE_LINUX_UPLOAD_CONTROL_WRITE_STATUS_OFFSET 0x06

/**
 * - STATUS_READY:              Client ready to perform a write operation. This is the only state in which a write
 *                              operation can be triggered. The client will report this state when all write operations
 *                              are done and the Write register is set to 0.
 *
 * - STATUS_BUSY:               Client currently performing a write operation.
 *
 * - STATUS_SUCCESS:            The previous write operation succeeded.
 *
 * - STATUS_FAIL_DEVICE_ERROR:  The previous write failed because the device encountered an error. The buffer page will
 *                              contain a string description of the error, and the DATA_LENGTH register will be updated
 *                              to contain the length of that error message in bytes.
 *
 * - STATUS_FAIL_BAD_CRC:       The previous write failed because the provided CRC did not match the actual CRC of the
 *                              data in the Buffer page.
 */

#define CANMORE_LINUX_UPLOAD_WRITE_STATUS_READY 0
#define CANMORE_LINUX_UPLOAD_WRITE_STATUS_BUSY 1  // TODO: needed?
#define CANMORE_LINUX_UPLOAD_WRITE_STATUS_SUCCESS 2
#define CANMORE_LINUX_UPLOAD_WRITE_STATUS_FAIL_DEVICE_ERROR 3
#define CANMORE_LINUX_UPLOAD_WRITE_STATUS_FAIL_BAD_CRC 4

#ifdef __cplusplus
}
#endif

#endif
