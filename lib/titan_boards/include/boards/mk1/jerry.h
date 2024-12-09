#ifndef BOARDS__MK1__JERRY_H_
#define BOARDS__MK1__JERRY_H_

#include "boards/pico.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// I2C Bus
#define DEPTH_I2C 0
#define DEPTH_I2C_SDA_PIN 8
#define DEPTH_I2C_SCL_PIN 9

// On-Board LED Pin
#define FAULT_LED_PIN PICO_DEFAULT_LED_PIN

#endif
