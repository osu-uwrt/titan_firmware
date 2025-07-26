#ifndef BOARDS__LILBOAT__HBRIDGE_BOARD_H_
#define BOARDS__LILBOAT__HBRIDGE_BOARD_H_

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
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_ESC_BOARD0

#define NSLEEP_PIN 25
#define DRVOFF_PIN 12

#define H0PH_PIN 0
#define H0EN_PIN 1
#define H0_INVERTED false

#define H1PH_PIN 2
#define H1EN_PIN 3
#define H1_INVERTED true

#define H2PH_PIN 4
#define H2EN_PIN 5
#define H2_INVERTED false

#define H3PH_PIN 6
#define H3EN_PIN 7
#define H3_INVERTED true

#define H4PH_PIN 8
#define H4EN_PIN 9
#define H4_INVERTED false

#define MOTOR_PORT_BRIDGE_NUM 0
#define MOTOR_STDB_BRIDGE_NUM 1
#define THRUSTER_PORT_BRIDGE_NUM 2
#define THRUSTER_STDB_BRIDGE_NUM 3
#define EXTRA_BRIDGE_NUM 4

#define MP_S0_PIN 23
#define MP_S1_PIN 28
#define MP_S2_PIN 29
#define MP_DATA_PIN 26

#define PUMP_SW_PIN 10

#define SERVO_PIN 24

#endif
