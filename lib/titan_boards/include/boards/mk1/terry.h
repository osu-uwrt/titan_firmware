#ifndef BOARDS__MK1__TERRY_H_
#define BOARDS__MK1__TERRY_H_

#include "boards/pico.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// 3V3                               ROUTE ME
// GND                               ROUTE ME

// I2C Bus
#define CONNECTOR_I2C 0
#define CONNECTOR_SDA_PIN 0  // ROUTE ME
#define CONNECTOR_SCL_PIN 1  // ROUTE ME

// Torpedos
#define COIL_1_PIN 10       // ROUTE ME
#define COIL_2_PIN 11       // ROUTE ME
#define COIL_3_PIN 12       // ROUTE ME
#define TORP_SEL_1_PIN 5    // ROUTE ME
#define TORP_SEL_2_PIN 6    // ROUTE ME
#define TORP_ARM_PIN 27     // ROUTE ME
#define TORP_CHARGE_LVL 28  // ROUTE ME

// Claw
#define CLAW_ENABLE_PIN 14
#define CLAW_MODE2_PIN 15
#define CLAW_DIRECTION_PIN 16
#define CLAW_CURRENT_PIN 26

// Dropper
#define DROPPER_1_PIN 21  // ROUTE ME
#define DROPPER_2_PIN 18  // ROUTE ME

#define BUILTIN_LED3_PIN 22

// On-Board LED Pin
#define FAULT_LED_PIN PICO_DEFAULT_LED_PIN

#endif
