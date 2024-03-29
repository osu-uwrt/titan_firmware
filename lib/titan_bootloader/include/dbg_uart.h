#ifndef DBG_UART_H_
#define DBG_UART_H_

void dbg_uart_init(void);
void dbg_uart_write(const char *s);
void dbg_uart_puts(const char *s);

#endif
