#ifndef _BOARDS_PUDDLES_BACKPLANE_H
#define _BOARDS_PUDDLES_BACKPLANE_H

// For board detection
#define PUDDLES_BACKPLANE

// ==============================
// Board Address Definitions
// ==============================

#define ETHERNET_IP       {192, 168,   1,  42}
#define ETHERNET_MASK     {255, 255, 255,   0}
#define ETHERNET_GATEWAY  {192, 168,   1,   1}

// ==============================
// Board Pin Definitions
// ==============================

// Dedicated I2C Header
#define BUILTIN_I2C_HW   i2c0
#define BUILTIN_SDA_PIN     0
#define BUILTIN_SCL_PIN     1

// On-Board LED Pins
#define LED_RGB_R_PIN   29
#define LED_RGB_G_PIN   28
#define LED_RGB_B_PIN   27

// On-Board Ethernet phy
#define ETH_SPI_HW        0
#define ETH_CLK_PIN       2
#define ETH_MOSI_PIN      3
#define ETH_MISO_PIN      0
#define ETH_CS_PIN        1
#define ETH_INT_PIN       4
#define ETH_RST_PIN       5

// I2C comm lines
#define SENSOR_I2C_HW   i2c0
#define SENSOR_SCL_PIN  9
#define SENSOR_SDA_PIN  8
#define BOARD_I2C_HW    i2c1
#define BOARD_SCL_PIN   7
#define BOARD_SDA_PIN   6

// Switch sense pins -- active low
#define KILL_SW_SENSE   12
#define AUX_SW_SENSE    11

// Power control pins -- active high
#define PWR_CTL_CPU     26
#define PWR_CTL_ACC     10

// ESC Comm pins -- PWM or DSHOT
#define THRUSTER_1_PIN      25
#define THRUSTER_2_PIN      24
#define THRUSTER_3_PIN      23
#define THRUSTER_4_PIN      22
#define THRUSTER_5_PIN      21
#define THRUSTER_6_PIN      20
#define THRUSTER_7_PIN      19
#define THRUSTER_8_PIN      18

// ==============================
// Pico Default Defines
// ==============================

// UART Definition
// NOTE: UART will conflict with the default I2C lines
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 0
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 1
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN PIN_RGB_R
#endif
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C BUILTIN_I2C_HW
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN BUILTIN_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN BUILTIN_SCL_PIN
#endif

// --- SPI ---
// This is reserved for the ethernet slot
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI ETH_SPI_HW
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN ETH_CLK_PIN
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN ETH_MOSI_PIN
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN ETH_MISO_PIN
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN ETH_CS_PIN
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#endif