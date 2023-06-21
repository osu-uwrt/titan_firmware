#ifndef BOARDS__MK2__PUDDLES_BACKPLANE_H_
#define BOARDS__MK2__PUDDLES_BACKPLANE_H_

// For board detection
#define PUDDLES_BACKPLANE

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// ==============================
// Board Address Definitions
// ==============================

#define ETHERNET_IP       {192, 168,   1,  42}
#define ETHERNET_MASK     {255, 255, 255,   0}
#define ETHERNET_GATEWAY  {192, 168,   1,   1}
#define ETHERNET_PORT 1337

// ==============================
// Board Pin Definitions
// ==============================

// On-Board LED Pins
#define STATUS_LEDR_PIN   29
#define STATUS_LEDG_PIN   28
#define STATUS_LEDB_PIN   27

// On-Board Ethernet phy
#define ETH_SPI         0
#define ETH_RST_PIN     5
#define ETH_INT_PIN     4
#define ETH_MOSI_PIN    3
#define ETH_CLK_PIN     2
#define ETH_CS_PIN      1
#define ETH_MISO_PIN    0

// I2C comm lines
#define SENSOR_I2C      0
#define SENSOR_SCL_PIN  9
#define SENSOR_SDA_PIN  8
#define BOARD_I2C       1
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
#define RP2040_DEBUG_UART     0
#define RP2040_DEBUG_TX_PIN   16

#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART RP2040_DEBUG_UART
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN RP2040_DEBUG_TX_PIN
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN PIN_RGB_R
#endif
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C         BOARD_I2C
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN BOARD_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN BOARD_SCL_PIN
#endif

// --- SPI ---
// This is reserved for the ethernet slot
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI ETH_SPI
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