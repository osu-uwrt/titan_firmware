#ifndef BOARDS_SBH_MCU_H
#define BOARDS_SBH_MCU_H

#include "boards/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

#define PERIPH_SDA_PIN      0
#define PERIPH_SCL_PIN      1
#define LED_R_PIN           2
#define LED_Y_PIN           3
#define LED_G_PIN           4

#define BMS_SDA_PIN         6
#define BMS_SCL_PIN         7
#define BMS_WAKE_PIN        8
#define PWR_CTRL_PIN        9
#define SWITCH_SIGNAL_PIN   10

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN PERIPH_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN PERIPH_SCL_PIN
#endif

#endif