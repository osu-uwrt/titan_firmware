#ifndef BOARDS__MERCURY__PO_HALF_BOARD_H_
#define BOARDS__MERCURY__PO_HALF_BOARD_H_

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
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_POWER_BOARD

#define PHYS_KILLSWITCH_PIN 2
#define SOFT_KILLSWITCH_PIN 4

#define AUX_SWITCH_PIN 3

#define SOLENOID_3_SW_PIN 1

// i2c bus with temp sensor, adc, depth sensor
#define BOARD_I2C 0
#define BOARD_SDA_PIN 24
#define BOARD_SCL_PIN 25

#define LEAK_DETECT_PIN 9  // DONT USE

#endif  // BOARDS__MERCURY__PO_HALF_BOARD_H_
