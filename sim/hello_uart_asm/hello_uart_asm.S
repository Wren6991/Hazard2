.option push
.option norelax
.option norvc

#include "platform_defs.h"

.section .vectors, "ax"

.global _start
_start:
	jal uart_init
	la a0, hello_text
	jal uart_puts
halt:
	j halt

.section .rodata

hello_text:
	.asciz "Hello, world from Hazard2 soft UART!\nRunning at 115200 baud.\n"

.option pop
