#ifndef _BACKPLANE_BREAKOUT_BOARD_H
#define _BACKPLANE_BREAKOUT_BOARD_H

#include "boards/pico.h"

// Make sure multiple boards aren't included
// This might happen if one board is #included and another one is specified as the defined board in CMakeLists
#ifdef UWRT_BOARD_DEFINED
#error Multiple board types included
#endif
#define UWRT_BOARD_DEFINED


#define FAULT_LED_PIN 4

#endif