#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/uart.h"
#include "pico/binary_info.h"

#if defined(PICO_DEFAULT_UART) && defined(PICO_DEFAULT_UART_TX_PIN)

#define debug_uart_hw (__CONCAT(__CONCAT(uart, PICO_DEFAULT_UART), _hw))
#ifndef PICO_DEFAULT_UART_BAUD_RATE
#define PICO_DEFAULT_UART_BAUD_RATE 115200
#endif

void dbg_uart_init(void) {
    bi_decl_if_func_used(bi_program_feature("Bootloader UART stdout"));

    // Make sure that UART hasn't been enabled yet
    hard_assert(!(debug_uart_hw->cr & UART_UARTCR_UARTEN_BITS));

    // Set UART Baud Rate
    uint32_t baud_rate_div = (8 * clock_get_hz(clk_peri) / PICO_DEFAULT_UART_BAUD_RATE);
    uint32_t baud_ibrd = baud_rate_div >> 7;
    uint32_t baud_fbrd;

    if (baud_ibrd == 0) {
        baud_ibrd = 1;
        baud_fbrd = 0;
    }
    else if (baud_ibrd >= 65535) {
        baud_ibrd = 65535;
        baud_fbrd = 0;
    }
    else {
        baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;
    }

    debug_uart_hw->ibrd = baud_ibrd;
    debug_uart_hw->fbrd = baud_fbrd;
    // We need to write LCR to latch the baud rate, but we're doing that in the next step anyways

    // Set UART Line Control (must be done before enable, if re-configuration required, extra steps must be taken)
    // But, to keep code size down, we're just going to write once since we know UART was reset during runtime init
    // Configuration:
    // 8 bit data (WLEN = b11)
    // 1 stop bit (STP2 = 0)
    // No Parity (PEN = 0, EPS = 0, SPS = 0)
    // No FIFOs (FEN = 0)
    // Normal Operation/Don't send break (BRK = 0)
    debug_uart_hw->lcr_h = (0b11 << UART_UARTLCR_H_WLEN_LSB);

    // Enable UART transmit only
    debug_uart_hw->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS;
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
}

void dbg_uart_write(const char *s) {
    while (*s) {
        // Wait for any pending bytes to send
        while (debug_uart_hw->fr & UART_UARTFR_TXFF_BITS)
            tight_loop_contents();

        // Send the next byte
        debug_uart_hw->dr = *s++;
    }
}

void dbg_uart_puts(const char *s) {
    dbg_uart_write(s);
    dbg_uart_write("\r\n");
}

#else

#warning No UART hardware found - Could not enable bootloader debug console

void dbg_uart_init(void) {}
void dbg_uart_write(const char *s) {
    (void) s;
}
void dbg_uart_puts(const char *s) {
    (void) s;
}

#endif
