#ifndef BOARDS_MK2_ESC_BOARD_H
#define BOARDS_MK2_ESC_BOARD_H

#include "boards/block/rp2040_can_block.h"

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

#endif