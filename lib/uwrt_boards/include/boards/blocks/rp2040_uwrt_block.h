#ifndef RP_2040_UWRT_BLOCK_H
#define RP_2040_UWRT_BLOCK_H

#define RP2040_DEBUG_UART     0
#define RP2040_DEBUG_TX_PIN   16

#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART RP2040_DEBUG_UART
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN RP2040_DEBUG_TX_PIN
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#ifndef PICO_RP2040_B0_SUPPORTED
#define PICO_RP2040_B0_SUPPORTED 0
#endif

#endif