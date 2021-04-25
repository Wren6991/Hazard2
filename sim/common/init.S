#define IO_BASE 0x80000000
#define IO_PRINT_CHAR (IO_BASE + 0x0)
#define IO_PRINT_U32  (IO_BASE + 0x4)
#define IO_EXIT       (IO_BASE + 0x8)

.option push
.option norelax
.option norvc

.section .vectors

// ----------------------------------------------------------------------------
// Reset handler


.reset_handler:
	la sp, __stack_top
	la t0, .vector_table
	csrw mtvec, t0

	// newlib _start expects argc, argv on the stack. Leave stack 16-byte aligned.
	addi sp, sp, -16
	li a0, 1
	sw a0, (sp)
	la a0, progname
	sw a0, 4(sp)

	jal _start
	j .halt

.global _exit
_exit:
	li a1, IO_EXIT
	sw a0, (a1)

.global _sbrk
_sbrk:
	la a1, heap_ptr
	lw a2, (a1)
	add a0, a0, a2
	sw a0, (a1)
	mv a0, a2
	ret

heap_ptr:
	.word _end

.global .halt
.halt:
	j .halt

progname:
	.asciz "hazard2-testbench"

// You can relax now
.option pop
