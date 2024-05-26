#ifndef BOARDS__MK2__POWER_BOARD_H_
#define BOARDS__MK2__POWER_BOARD_H_

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

#define AUX_SWITCH_PIN 0
#define PHYS_KILLSWITCH_PIN 1
#define SOFT_KILLSWITCH_PIN 2

#define PORT_STAT_PIN 10
#define STBD_STAT_PIN 11

#define DYNAMIXEL_CHECK_PIN 26
#define DYNAMIXEL_PWM_PIN 27

#define FAN_SWITCH_PIN 28
#define FAN_TACH_PIN 29

#define BOARD_I2C 0
#define BOARD_SDA_PIN 4
#define BOARD_SCL_PIN 5

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C BOARD_I2C
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN BOARD_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN BOARD_SCL_PIN
#endif

#endif
