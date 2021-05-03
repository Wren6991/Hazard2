#define IO_BASE 0x80000000
#define IO_PRINT_CHAR (IO_BASE + 0x0)
#define IO_PRINT_U32  (IO_BASE + 0x4)
#define IO_EXIT       (IO_BASE + 0x8)

.option push
.option norelax

.section .vectors

// ----------------------------------------------------------------------------
// Reset handler


.reset_handler:
	la sp, __stack_top

	// newlib _start expects argc, argv on the stack. Leave stack 16-byte aligned.
	addi sp, sp, -16
	li a0, 1
	sw a0, (sp)
	la a0, progname
	sw a0, 4(sp)

	// _start will handle the .bss clearing, but we need to copy over .data init first.
	la a0, __data_load_src
	la a1, __data_load_dst
	la a2, __data_load_dst_end
	jal data_cpy_loop

	jal _start
	j .halt

data_cpy_loop:
	lw a3, (a0)
	sw a3, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
data_cpy:
	bltu a1, a2, data_cpy_loop
	ret

.global _sbrk
_sbrk:
	la a1, heap_ptr
	lw a2, (a1)
	add a0, a0, a2
	sw a0, (a1)
	mv a0, a2
	ret

.global _exit
_exit:
	li a1, IO_EXIT
	sw a0, (a1)
	// fall through:
.global .halt
.halt:
	j .halt

progname:
	.asciz "hazard2"

.section .data
.p2align 2
heap_ptr:
	.word _end

// You can relax now
.option pop
