#ifndef BOARDS__MK2__POAC_BOARD_H_
#define BOARDS__MK2__POAC_BOARD_H_

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
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_LED_BOARD

#define LEDC_DIN2_PIN 0
#define LEDC_DIN1_PIN 1
#define LEDC_NCS2_PIN 6
#define LEDC_PWM_CLK 7

#define LEDC_SPI 1
#define LEDC_NCS1_PIN 9
#define LEDC_SCK_PIN 10
#define LEDC_MOSI_PIN 11
#define LEDC_MISO_PIN 12

#define CONN_CHK_PIN 25
#define TEMP_SENSE_PIN 26

#endif
