#ifndef BOARDS__LILBOAT__PO_HALF_BOARD_H_
#define BOARDS__LILBOAT__PO_HALF_BOARD_H_

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

#define AUX_SWITCH_PIN 10
#define PHYS_KILLSWITCH_PIN 23
#define SOFT_KILLSWITCH_PIN 12

#define ORIN_SW_PIN 11

#define SOLENOID0_PIN 4
#define SOLENOID1_PIN 5
#define SOLENOID2_PIN 9

#define EXHAUST_SOLENOID_NUM 0
#define PRESSURE_SOLENOID_NUM 1
#define WATER_SOLENOID_NUM 2

#define ADC0_PIN 28
#define ADC1_PIN 29

#define TANK_PRESSURE_ADC_NUM 0
#define REGULATED_PRESSURE_ADC_NUM 1

#define DEPTH0_I2C 0
#define DEPTH0_SDA_PIN 24
#define DEPTH0_SCL_PIN 25

#define DEPTH1_I2C 1
#define DEPTH1_SDA_PIN 26
#define DEPTH1_SCL_PIN 27

#define WATER_DEPTH_NUM 0
#define REGHOUSING_DEPTH_NUM 1

#endif
