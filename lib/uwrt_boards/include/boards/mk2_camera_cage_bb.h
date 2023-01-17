#ifndef BOARDS_MK2_CAMERA_CAGE_BB_H
#define BOARDS_MK2_CAMERA_CAGE_BB_H

#include "boards/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

#define FAN_SW_PIN          0
#define TACH4_PIN           1
#define TACH3_PIN           2
#define TACH2_PIN           3
#define TACH1_PIN           4
#define PELT_SW_PIN         5
#define ORIN_SW_PIN         6
#define RGB_DATA_PIN        9

#define SDA_PIN 28
#define SCL_PIN 29
#define I2C_INSTANCE i2c0

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN SCL_PIN
#endif

#endif