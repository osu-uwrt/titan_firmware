#ifndef RP_2040_CAN_BLOCK_H
#define RP_2040_CAN_BLOCK_H

#include "boards/rp2040_uwrt_block.h"

#define STATUS_LEDR_PIN         13
#define STATUS_LEDG_PIN         14
#define STATUS_LEDB_PIN         15

#ifndef BUILTIN_LED3_PIN
#define BUILTIN_LED3_PIN STATUS_LEDB_PIN
#endif

#define FAULT_LED_PIN STATUS_LEDR_PIN
#define FAULT_LED_INVERTED      1

#define MCP2517FD_NCS_PIN       17
#define MCP2517FD_SCK_PIN       18
#define MCP2517FD_MOSI_PIN      19
#define MCP2517FD_MISO_PIN      20
#define MCP2517FD_CANCLK_PIN    21 
#define MCP2517FD_INT_PIN       22

#endif