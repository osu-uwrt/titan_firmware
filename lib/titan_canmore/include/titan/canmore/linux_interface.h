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
 */

// Page Number Definitions
#define CANMORE_LINUX_GEN_CONTROL_PAGE_NUM 0x00
#define CANMORE_LINUX_VERSION_STRING_PAGE_NUM 0x01
#define CANMORE_LINUX_TTY_CONTROL_PAGE_NUM 0x02
#define CANMORE_LINUX_TTY_TERMINAL_PAGE_NUM 0x03
#define CANMORE_LINUX_TTY_CMD_PAGE_NUM 0x04

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

#ifdef __cplusplus
}
#endif

#endif
