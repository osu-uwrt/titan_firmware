#ifndef BOARDS_MK2_ESC_BOARD_H
#define BOARDS_MK2_ESC_BOARD_H

#include "boards/blocks/rp2040_can_block.h"

// This defines which CAN bus this board is connected into
// The CAN busses defined in the corresponding robot definition files (rate, enable FD, etc.)
#define CAN_BUS_NAME INTERNAL_CAN
#define CAN_BUS_CLIENT_ID 4

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

#define ESC4_PWM_PIN        0
#define ESC4_TELEM_PIN      1
#define ESC3_PWM_PIN        2
#define ESC3_TELEM_PIN      3
#define ESC2_PWM_PIN        4
#define ESC2_TELEM_PIN      5
#define ESC1_PWM_PIN        6
#define ESC1_TELEM_PIN      7
#define VCC_MEAS_PIN        28
#define BOARD_DET_PIN       29

#define BOARD_NAMESPACE "esc_board"

#endif