.option push
.option norelax
.option norvc

#define SYS_CLK_KHZ 12000

#define LED_BASE 0x80000000
#define BLINK_PERIOD_MS 200
#define N_LEDS 5

.section .vectors, "ax"

.reset_handler:
	j _start

.section .text

1:
	addi a0, a0, -1
delay_cycles_times_3:
	bnez a0, 1b
	ret

.global _start
_start:
	// Initial LED mask
	li a1, 0x1

blink_loop:
	// Illuminate LEDs based on current mask
	li a0, LED_BASE
	add a0, a0, a1
	lbu a0, (a0)

	li a0, (SYS_CLK_KHZ * BLINK_PERIOD_MS) / 3
	jal delay_cycles_times_3

	// Advance to next LED
	slli a1, a1, 1
	srli a0, a1, N_LEDS
	beqz a0, blink_loop
	li a1, 1
	j blink_loop
	

.option pop
