#ifndef BOARDS__MK1__ACTUATOR_MK1_HAT_H_
#define BOARDS__MK1__ACTUATOR_MK1_HAT_H_

#include "boards/mk1/rp2040_eval_board.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// I2C Bus
#define CONNECTOR_I2C 0
#define CONNECTOR_SDA_PIN 24
#define CONNECTOR_SCL_PIN 25

// Torpedos
#define COIL_1_PIN 9
#define COIL_2_PIN 10
#define COIL_3_PIN 11
#define COIL_4_PIN 12
#define TORP_SEL_1_PIN 15
#define TORP_SEL_2_PIN 14
#define TORP_ARM_PIN 23
#define TORP_CHARGE_LVL 27

// Claw
#define CLAW_ENABLE_PIN 22
#define CLAW_MODE2_PIN 29
#define CLAW_DIRECTION_PIN 28
#define CLAW_CURRENT_PIN 26

// Dropper
#define DROPPER_1_PIN 17
#define DROPPER_2_PIN 16

// On-Board LED Pin
#define FAULT_LED_PIN BUILTIN_LED1_PIN

#endif
