TEST = add-01
TEST_ARCH = I
BIN_ARCH = rv32i
SIM_EXEC = ../tb.py

CROSS_PREFIX = /opt/riscv/bin/riscv32-unknown-elf-

TEST_BIN_NAME := $(TEST_ARCH)-$(TEST)-on-$(BIN_ARCH)
TEST_SRC := riscv-arch-test/riscv-test-suite/rv32i_m/$(TEST_ARCH)/src/$(TEST).S
TEST_VEC := riscv-arch-test/riscv-test-suite/rv32i_m/$(TEST_ARCH)/references/$(TEST).reference_output

.PHONY: all testlist clean
all:
	mkdir -p tmp
	$(CROSS_PREFIX)gcc -I include -T memmap.ld -nostartfiles -march=$(BIN_ARCH) $(TEST_SRC) -DXLEN=32 -o tmp/$(TEST_BIN_NAME).elf
	$(CROSS_PREFIX)objdump -h tmp/$(TEST_BIN_NAME).elf > tmp/$(TEST_BIN_NAME).dis
	$(CROSS_PREFIX)objdump -d tmp/$(TEST_BIN_NAME).elf >> tmp/$(TEST_BIN_NAME).dis
	$(CROSS_PREFIX)objcopy -O binary tmp/$(TEST_BIN_NAME).elf tmp/$(TEST_BIN_NAME).bin
	$(SIM_EXEC) tmp/$(TEST_BIN_NAME).bin --dump 0x400000 0x401000 | tee tmp/$(TEST_BIN_NAME).log
	./compare_testvec tmp/$(TEST_BIN_NAME).log $(TEST_VEC)

testlist: 
	$(MAKE) TEST=add-01
	$(MAKE) TEST=addi-01
	$(MAKE) TEST=and-01
	$(MAKE) TEST=andi-01
	$(MAKE) TEST=auipc-01
	$(MAKE) TEST=beq-01
	$(MAKE) TEST=bge-01
	$(MAKE) TEST=bgeu-01
	$(MAKE) TEST=blt-01
	$(MAKE) TEST=bltu-01
	$(MAKE) TEST=bne-01
	$(MAKE) TEST=fence-01
	$(MAKE) TEST=jal-01
	$(MAKE) TEST=jalr-01
	$(MAKE) TEST=lb-align-01
	$(MAKE) TEST=lbu-align-01
	$(MAKE) TEST=lh-align-01
	$(MAKE) TEST=lhu-align-01
	$(MAKE) TEST=lui-01
	$(MAKE) TEST=lw-align-01
	$(MAKE) TEST=or-01
	$(MAKE) TEST=ori-01
	$(MAKE) TEST=sb-align-01
	$(MAKE) TEST=sh-align-01
	$(MAKE) TEST=sll-01
	$(MAKE) TEST=slli-01
	$(MAKE) TEST=slt-01
	$(MAKE) TEST=slti-01
	$(MAKE) TEST=sltiu-01
	$(MAKE) TEST=sltu-01
	$(MAKE) TEST=sra-01
	$(MAKE) TEST=srai-01
	$(MAKE) TEST=srl-01
	$(MAKE) TEST=srli-01
	$(MAKE) TEST=sub-01
	$(MAKE) TEST=sw-align-01
	$(MAKE) TEST=xor-01
	$(MAKE) TEST=xori-01

clean:
	rm -rf tmp/
