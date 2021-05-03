#ifndef _SOFT_UART_H
#define _SOFT_UART_H

void uart_putc(char c);

void uart_puts(const char *s);

void uart_init(void);

#endif
