#ifndef BOARDS__MK1__BACKPLANE_BREAKOUT_BOARD_EVAL_HAT_H_
#define BOARDS__MK1__BACKPLANE_BREAKOUT_BOARD_EVAL_HAT_H_

#include "boards/mk1/rp2040_eval_board.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// ==============================
// Board Address Definitions
// ==============================

#define ETHERNET_IP                                                                                                    \
    { 192, 168, 1, 43 }

// ==============================
// Board Pin Definitions
// ==============================

// I2C Busses
#define SENSOR_I2C BUILTIN_I2C
#define SENSOR_SDA_PIN BUILTIN_SDA_PIN
#define SENSOR_SCL_PIN BUILTIN_SCL_PIN
#define BOARD_I2C 1
#define BOARD_SDA_PIN 26
#define BOARD_SCL_PIN 27

// Thrusters
#define THRUSTER_1_PIN 23
#define THRUSTER_2_PIN 24
#define THRUSTER_3_PIN 21
#define THRUSTER_4_PIN 20
#define THRUSTER_5_PIN 18
#define THRUSTER_6_PIN 16
#define THRUSTER_7_PIN 14
#define THRUSTER_8_PIN 19

// Switches
#define KILL_SWITCH_PIN 25
#define AUX_SWITCH_PIN 29

// Power Control Pins
#define PELTIER_CTRL_PIN 22
// #define REG_12_CTRL_PIN   No reg 12 on board
#define REG_5_CTRL_PIN 15
#define MOBO_CTRL_PIN 17
#define LIGHTS_1_PIN 28
// #define LIGHTS_2_PIN      No lights 2 on board

// On-Board LED Pin
#define FAULT_LED_PIN BUILTIN_LED1_PIN

#endif
