#include "soft_uart.h"

int main() {
	uart_init();
	uart_puts("\nBooted from flash.\n");
	uart_puts("Hello, world from C!\n");
}
