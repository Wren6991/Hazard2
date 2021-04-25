#define IO_BASE 0x80000000
#define IO_PRINT_CHAR (IO_BASE + 0x0)
#define IO_PRINT_U32  (IO_BASE + 0x4)
#define IO_EXIT       (IO_BASE + 0x8)

.option push
.option norelax
.option norvc

.section .vectors

.global _start
_start:
	la a0, hello_text
	li a1, IO_PRINT_CHAR
	j 2f
1:
	sb a2, (a1)
	addi a0, a0, 1
2:
	lbu a2, (a0)
	bnez a2, 1b

	li a1, IO_EXIT
	li a0, 123
	sw a0, (a1)
halt:
	j halt

hello_text:
	.asciz "Hello, world from Hazard2 + nMigen!\n"

.option pop
