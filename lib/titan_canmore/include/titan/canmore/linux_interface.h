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
#define CANMORE_LINUX_FILE_CONTROL_PAGE_NUM 0x06

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
 * File Control
 * ==============
 * Contains registers to control the data stored in the Upload Buffer page.
 *
 *       +-----------------+
 * 0x00: | Filename Length |  WO
 *       +-----------------+
 * 0x01  |   Data Length   | RW
 *       +-----------------+
 * 0x02: |      CRC32      | WO
 *       +-----------------+
 * 0x03: |      Clear      | WO
 *       +-----------------+
 * 0x04: |   Read Offset   | WO
 *       +-----------------+
 * 0x05: |    File Mode    | RW
 *       +-----------------+
 * 0x06: |    Operation    | WO
 *       +-----------------+
 * 0x07: |   Write Status  | RO
 *       +-----------------+
 *
 * Filename Length: The length of the filename in bytes. The data should start at the beginning of the next register.
 *
 * Data Length:     The number of bytes (not regs) to write from the buffer into the file.
 *
 * CRC32:           A 32-bit CRC value for a target which is dependent on the operation. If the operation is WRITE, then
 *                  this should be the 32-bit CRC of the data in the file buffer page. If the operation is CHECK_CRC,
 *                  then this value should be the 32-bit CRC of the file being checked.
 *
 * Clear:           If set to 0 (false), a write action will append the file data contents of the File Buffer page to
 *                  the end of the file. Otherwise, a write action will delete the file data contents of the file on the
 *                  remote device before writing the contents of the File Buffer page into it
 *
 * File Mode:       The desired mode of the file.
 *
 * Read Offset:     When a read action is triggered, this register describes the offset in bytes of the data in the file
 *                  to read.
 *
 * Operation:       Writing a value to this register will trigger an operation corresponding to the value written. Valid
 *                  values are listed below under FILE OPERATIONS.
 *
 * Write Status:    Contains the current status of the remote device filewriter. Possible values are described below
 *                  where the status macros are defined.
 */

#define CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET 0x00
#define CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET 0x01
#define CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET 0x02
#define CANMORE_LINUX_FILE_CONTROL_CLEAR_OFFSET 0x03
#define CANMORE_LINUX_FILE_CONTROL_READ_OFFSET_OFFSET 0x04
#define CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET 0x05
#define CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET 0x06
#define CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET 0x07

/**
 * FILE OPERATIONS
 * - OPERATION_NOP:             No operation. Set the operation register to this value to put the file controller into a
 *                              READY state.
 *                                  Required registers: None
 *
 * - OPERATION_WRITE:           Write the contents of the file buffer into a file.
 *                                  Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *                                  - DATA_LENGTH
 *                                  - CRC: Contains the CRC32 value of all data in the file buffer.
 *                                  - CLEAR
 *
 * - OPERATION_READ:            Read a portion of a file into the file buffer. Populates the file buffer, DATA_LENGTH,
 *                              and CRC with the expected CRC value of the data in the file buffer. Populates STATUS
 *                              with SUCCESS unless a device error happens in which case STATUS will be populated with
 *                              DEVICE_ERROR.
 *                              Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *                                  - CLEAR: should be true if the CRC should be cleared
 *
 * - OPERATION_SET_MODE:        Set the file mode to using the value in the FILE_MODE register.
 *                                  Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *                                  - FILE_MODE
 *
 * - OPERATION_GET_MODE         Get a remote file mode to the FILE_MODE register.
 *                                  Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *
 * - OPERATION_GET_FILE_LEN     Get the length of a file to the DATA_LENGTH register.
 *                                  Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *
 * - OPERATION_CHECK_CRC:       Check the CRC32 value of the remote file against the one provided in the CRC register.
 *                              If the CRC is good, then the status will be set to SUCCESS. Otherwise the status will be
 *                              set to BAD_CRC.
 *                                  Required registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to check.
 *                                  - CRC: Contains the CRC32 of the file being checked.
 *
 * - OPERATION_CD               Change the working directory for file transfer to a relative or absolute directory
 *                              specified in the file whose name is provided in the file buffer. Can return SUCCESS OR
 *                              FAIL_DEVICE_ERROR
 *                                  Requred registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to read.
 *
 * - OPERATION_LS               List the contents of the directory specified in the file whose name is provided in the
 *                              file buffer. The directory contents will be written to that file as well. Can return
 *                              SUCCESS or FAIL_DEVICE_ERROR Requred registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 *
 * - OPERATION_PWD              Write the full current working directory to the file whose name is provided in the file
 *                              buffer. Can return SUCCESS or FAIL_DEVICE_ERROR
 *                                  Requred registers:
 *                                  - FILENAME_LENGTH
 *                                      - File buffer must be populated at the beginning with this many bytes of data
 *                                        describing the name of the file to write.
 */

#define CANMORE_LINUX_FILE_OPERATION_NOP 0
#define CANMORE_LINUX_FILE_OPERATION_WRITE 1
#define CANMORE_LINUX_FILE_OPERATION_READ 2
#define CANMORE_LINUX_FILE_OPERATION_SET_MODE 3
#define CANMORE_LINUX_FILE_OPERATION_GET_MODE 4
#define CANMORE_LINUX_FILE_OPERATION_GET_FILE_LEN 5
#define CANMORE_LINUX_FILE_OPERATION_CHECK_CRC 6
#define CANMORE_LINUX_FILE_OPERATION_CD 7
#define CANMORE_LINUX_FILE_OPERATION_LS 8
#define CANMORE_LINUX_FILE_OPERATION_PWD 9

/**
 * FILE STATUSES
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

#define CANMORE_LINUX_FILE_STATUS_READY 0
#define CANMORE_LINUX_FILE_STATUS_BUSY 1
#define CANMORE_LINUX_FILE_STATUS_SUCCESS 2
#define CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR 3
#define CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC 4

#ifdef __cplusplus
}
#endif

#endif
