#ifndef BOARDS__MK2__ESC_BOARD_H_
#define BOARDS__MK2__ESC_BOARD_H_

#include "boards/mk2/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// This defines which CAN bus this board is connected into
// The CAN bus is defined in the corresponding robot definition files (rate, enable FD, etc.)
#define CAN_BUS_NAME INTERNAL_CAN
// This defines the client ID for this board on that bus
// These are defined in the titan_canmore/.../client_ids.h header file (and implicity included by titan_boards.cmake)
// Ensure that the bus that the client id below belongs to matches the bus selected above
#define CAN_BUS_BOARD0_CLIENT_ID CANMORE_CLIENT_ID_ESC_BOARD0
#define CAN_BUS_BOARD1_CLIENT_ID CANMORE_CLIENT_ID_ESC_BOARD1

// Define custom client lookup for the bootloader (since we need to detect which board we're on)
#define TITAN_BOOTLOADER_CUSTOM_CLIENT_LOOKUP "can_bl_custom_id/mk2_esc_board.h"

#define ESC4_PWM_PIN 0
#define ESC4_TELEM_PIN 1
#define ESC3_PWM_PIN 2
#define ESC3_TELEM_PIN 3
#define ESC2_PWM_PIN 4
#define ESC2_TELEM_PIN 5
#define ESC1_PWM_PIN 6
#define ESC1_TELEM_PIN 7
#define VCC_MEAS_PIN 28
#define BOARD_DET_PIN 29

#endif
