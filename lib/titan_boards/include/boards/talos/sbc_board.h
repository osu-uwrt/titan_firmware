#ifndef BOARDS__TALOS__SBC_BOARD_H_
#define BOARDS__TALOS__SBC_BOARD_H_

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
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_SBC_BOARD

#define AUDIOIN_PIN 0
#define BCLK_PIN 1
#define LRCLK_PIN 2

#define SCRNREST_PIN 4
#define IRQ_PIN 5
#define TEARING_PIN 6

#define CSX_PIN 9
#define SCLK_PIN 10
#define STX_PIN 11
#define SRX_PIN 12

#define DSWITCH_PIN 23
#define PROCHOT_PIN 24
#define CHRG_OK_PIN 25
#define OTG_PIN 26
#define OR_CONTROL 27

#define BOARD_I2C 0
#define BOARD_SDA_PIN 28
#define BOARD_SCL_PIN 29

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
