#ifndef _BACKPLANE_BREAKOUT_BOARD_H
#define _BACKPLANE_BREAKOUT_BOARD_H

#include "boards/pico.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// I2C Busses
#define SENSOR_I2C        0
#define SENSOR_SDA_PIN    0
#define SENSOR_SCL_PIN    1
#define BOARD_I2C         1
#define BOARD_SDA_PIN    26
#define BOARD_SCL_PIN    27

// Thrusters
#define THRUSTER_1_PIN    7
#define THRUSTER_2_PIN    8
#define THRUSTER_3_PIN   21
#define THRUSTER_4_PIN   20
#define THRUSTER_5_PIN   18
#define THRUSTER_6_PIN   16
#define THRUSTER_7_PIN   14
#define THRUSTER_8_PIN   19

// Switches
#define KILL_SWITCH_PIN   2
#define AUX_SWITCH_PIN    3

// Power Control Pins
#define PELTIER_CTRL_PIN 22
#define REG_12_CTRL_PIN   5
#define REG_5_CTRL_PIN   15
#define MOBO_CTRL_PIN    17
#define LIGHTS_1_PIN     28
#define LIGHTS_2_PIN      6

// On-Board LED Pin
#define FAULT_LED_PIN     4

// On-Board Ethernet Pins
#define ETH_SPI           1
#define ETH_RST_PIN       9
#define ETH_CLK_PIN      10
#define ETH_MOSI_PIN     11
#define ETH_MISO_PIN     12
#define ETH_CS_PIN       13

#endif