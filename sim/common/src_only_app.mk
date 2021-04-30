ifndef SRCS
$(error Must define list of test sources as SRCS)
endif

ifndef APP
$(error Must define application name as APP)
endif

CCFLAGS      ?=
LDSCRIPT     ?= ../common/memmap.ld
CROSS_PREFIX ?= riscv32-unknown-elf-
TBDIR        ?= ..
INCDIR       ?= ../common
MAX_CYCLES   ?= 100000

###############################################################################

.SUFFIXES:
.PHONY: all run view clean

all: run

run: $(APP).bin
	$(TBDIR)/tb.py $(APP).bin $(APP).vcd --cycles $(MAX_CYCLES)

view: run
	gtkwave $(APP)_run.vcd

bin: $(APP).bin

clean:
	rm -f $(APP).elf $(APP).bin $(APP).dis $(APP)_run.vcd

###############################################################################

$(APP).bin: $(APP).elf
	$(CROSS_PREFIX)objcopy -O binary $^ $@
	$(CROSS_PREFIX)objdump -h $(APP).elf > $(APP).dis
	$(CROSS_PREFIX)objdump -d $(APP).elf >> $(APP).dis

$(APP).elf: $(SRCS) $(wildcard %.h)
	$(CROSS_PREFIX)gcc $(CCFLAGS) $(SRCS) -T $(LDSCRIPT) $(addprefix -I,$(INCDIR)) -o $(APP).elf
