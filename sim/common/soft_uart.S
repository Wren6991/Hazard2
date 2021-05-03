#include "platform_defs.h"

// Soft UART TX. Assumes that taken branches are 2 cycles, loads are 2 cycles,
// all other instructions 1 cycle. Does not use any stack (just lots of
// caller-save registers). Should be loaded to RAM for deterministic timing:

.section .time_critical, "a"

1:
	addi a2, a2, -1
baud_delay:
	bgez a2, 1b
	ret


.global uart_putc
uart_putc:
	mv t0, ra
	li a5, CLK_SYS_KHZ * 1000 / (3 * UART_BAUD) - 5
	li a4, UART_BASE
	// Preload start/stop bit
	ori a0, a0, 0x100
	slli a0, a0, 1
	// Send 10 bits. Each loop period is (14 + 3 * a5) cycles.
	li a3, 10
1:
	andi a1, a0, 1
	srli a0, a0, 1
	add a1, a1, a4
	lbu x0, (a1)
	mv a2, a5
	jal baud_delay
	addi a3, a3, -1
	bnez a3, 1b

	jr t0


// Remaining code is non-timing-critical.
.section .text

.global uart_puts
uart_puts:
	mv t1, ra
	mv t2, a0
	la t3, uart_putc
	j 2f
1:
	jalr t3
	addi t2, t2, 1
2:
	lbu a0, (t2)
	bnez a0, 1b

	jr t1

.global uart_init
uart_init:
	// Drive TX high, then wait 1 ms.
	li a0, UART_BASE
	lbu x0, 1(a0)
	li a0, CLK_SYS_KHZ / 3
	j 2f
1:
	addi a0, a0, -1
2:
	beqz a0, 1b
	ret
