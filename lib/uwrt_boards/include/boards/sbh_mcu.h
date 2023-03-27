#ifndef BOARDS_SBH_MCU_H
#define BOARDS_SBH_MCU_H

#include "boards/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

#define CAN_BUS_NAME EXTERNAL_CAN
#define CAN_BUS_PORT_CLIENT_ID 1
#define CAN_BUS_STBD_CLIENT_ID 2

// Define custom client lookup for the bootloader (since we need to detect which board we're on)
#define TITAN_BOOTLOADER_CUSTOM_CLIENT_LOOKUP "can_bl_custom_id/sbh_mcu.h"

#define PERIPH_SDA_PIN      0
#define PERIPH_SCL_PIN      1
#define LED_R_PIN           2
#define LED_Y_PIN           3
#define LED_G_PIN           4

// Note that the bms i2c bus is reversed from the pin-mux
// I2C will need to be implemented with PIO on this board
#define BMS_SDA_PIN         7
#define BMS_SCL_PIN         6
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