#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

#if defined(uart_default) && defined(PICO_DEFAULT_UART_TX_PIN)

void dbg_uart_init(void) {
    bi_decl_if_func_used(bi_program_feature("Bootloader UART stdout"));

    uart_init(uart_default, PICO_DEFAULT_UART_BAUD_RATE);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
}

void dbg_uart_write(const char *s) {
    uart_puts(uart_default, s);
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
