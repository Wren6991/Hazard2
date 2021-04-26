from nmigen import *
from nmigen.asserts import *
from enum import IntEnum

XLEN = 32

class ALUOp(IntEnum):
	ADD   = 0x0 
	SUB   = 0x1 
	LT    = 0x2
	LTU   = 0x4
	AND   = 0x6
	OR    = 0x7
	XOR   = 0x8
	SRL   = 0x9
	SRA   = 0xa
	SLL   = 0xb

class RVOpc(IntEnum):
	LOAD     = 0b00_000
	MISC_MEM = 0b00_011
	OP_IMM   = 0b00_100
	AUIPC    = 0b00_101
	STORE    = 0b01_000
	OP       = 0b01_100
	LUI      = 0b01_101
	BRANCH   = 0b11_000
	JALR     = 0b11_001
	JAL      = 0b11_011
	SYSTEM   = 0b11_100

def msb(val):
	return val[-1]

# Sorry my brain is wired this way now
def RCat(*varg):
	return Cat(reversed(varg))

def imm_i(instr):
	return RCat(Repl(msb(instr), 20), instr[20:])

def imm_s(instr):
	return RCat(Repl(msb(instr), 20), instr[25:], instr[7:12])

def imm_b(instr):
	return RCat(Repl(msb(instr), 20), instr[7], instr[25:31], instr[8:12], C(0, 1))

def imm_u(instr):
	return RCat(instr[12:], C(0, 12))

def imm_j(instr):
	return RCat(Repl(msb(instr), 12), instr[12:20], instr[20], instr[21:31], C(0, 1))

class Hazard2ALU(Elaboratable):
	def __init__(self):
		self.i0    = Signal(XLEN)
		self.i1    = Signal(XLEN)
		self.op    = Signal(Shape.cast(ALUOp))
		self.take4 = Signal()
		self.cmp   = Signal()
		self.o     = Signal(XLEN)

	def elaborate(self, platform):
		m = Module()
		adder = (Mux(self.op != ALUOp.ADD, self.i0 - self.i1, self.i0 + self.i1) - self.take4 * 4)[:XLEN]
		less_than = Mux(msb(self.i0) == msb(self.i1), msb(adder),
			Mux(self.op == ALUOp.LTU, msb(self.i1), msb(self.i0))
		)
		m.d.comb += self.cmp.eq(Mux(self.op == ALUOp.SUB, self.i0 == self.i1, less_than))

		# Bitwise ops can be implemented as a single rank of LUT4s. Try to encourage this.
		bitwise = Signal(XLEN)
		with m.Switch(self.op[0:2]):
			with m.Case(ALUOp.AND & 0x3):
				m.d.comb += bitwise.eq(self.i0 & self.i1)
			with m.Case(ALUOp.OR & 0x3):
				m.d.comb += bitwise.eq(self.i0 | self.i1)
			with m.Case():
				m.d.comb += bitwise.eq(self.i0 ^ self.i1)

		with m.Switch(self.op):
			with m.Case(ALUOp.ADD):
				m.d.comb += self.o.eq(adder)
			with m.Case(ALUOp.SUB):
				m.d.comb += self.o.eq(adder)
			with m.Case(ALUOp.LT):
				m.d.comb += self.o.eq(less_than)
			with m.Case(ALUOp.LTU):
				m.d.comb += self.o.eq(less_than)
			with m.Case(ALUOp.SRL):
				m.d.comb += self.o.eq(self.i0 >> self.i1[0:5])
			with m.Case(ALUOp.SRA):
				m.d.comb += self.o.eq((self.i0.as_signed() >> self.i1[0:5]).as_unsigned())
			with m.Case(ALUOp.SLL):
				m.d.comb += self.o.eq(self.i0 << self.i1[0:5])
			with m.Case():
				m.d.comb += self.o.eq(bitwise)
		return m


class Hazard2Regfile(Elaboratable):
	def __init__(self):
		self.raddr1 = Signal(5)
		self.raddr2 = Signal(5)
		self.ren    = Signal()
		self.rdata1 = Signal(XLEN)
		self.rdata2 = Signal(XLEN)
		self.waddr  = Signal(5)
		self.wdata  = Signal(XLEN)
		self.wen    = Signal()
		self.mem    = Memory(width=XLEN, depth=32, init=[0] * 32)

	def elaborate(self, platform):
		m = Module()
		m.submodules.wport = wport = self.mem.write_port()
		m.submodules.rport1 = rport1 = self.mem.read_port(transparent=False)
		m.submodules.rport2 = rport2 = self.mem.read_port(transparent=False)
		# nMigen/Yosys do not support read enable on read ports with transparency
		# enabled, so need to perform write-to-read bypass manually.

		prev_wdata = Signal(XLEN)
		forward_wdata_to_r1 = Signal()
		forward_wdata_to_r2 = Signal()
		next_is_forwarded = self.wen & self.ren & (self.waddr != 0)
		with m.If(next_is_forwarded):
			m.d.sync += prev_wdata.eq(self.wdata)
		with m.If(self.ren):
			m.d.sync += [
				forward_wdata_to_r1.eq(next_is_forwarded & (self.waddr == self.raddr1)),
				forward_wdata_to_r2.eq(next_is_forwarded & (self.waddr == self.raddr2))
			]

		m.d.comb += [
			rport1.addr.eq(self.raddr1),
			rport1.en.eq(self.ren),
			self.rdata1.eq(Mux(forward_wdata_to_r1, prev_wdata, rport1.data)),
			rport2.addr.eq(self.raddr2),
			rport2.en.eq(self.ren),
			self.rdata2.eq(Mux(forward_wdata_to_r2, prev_wdata, rport2.data)),
			wport.addr.eq(self.waddr),
			wport.data.eq(self.wdata),
			wport.en.eq(self.wen & (self.waddr != 0))
		]
		return m

class Hazard2CPU(Elaboratable):
	def __init__(self, reset_vector=0x0):
		self.reset_vector = reset_vector
		self.htrans = Signal(2)
		self.hwrite = Signal()
		self.hsize  = Signal(3)
		self.haddr  = Signal(XLEN)
		self.hwdata = Signal(XLEN)
		self.hrdata = Signal(XLEN)
		self.hready = Signal()

	def elaborate(self, platform):
		m = Module()

		stall = ~self.hready

		### Stage F ###

		i_dph_active = Signal()
		d_dph_active = Signal()
		d_dph_write  = Signal()
		d_dph_addr   = Signal(2)
		d_dph_size   = Signal(2)
		d_dph_signed = Signal()

		cir = Signal(32)
		cir_valid = Signal()
		load_rdata = Signal(XLEN)

		with m.If(i_dph_active & ~stall):
			m.d.sync += cir.eq(self.hrdata)

		with m.Switch(d_dph_size):
			with m.Case(2):
				m.d.comb += load_rdata.eq(self.hrdata)
			with m.Case(1):
				hword_rdata = self.hrdata.word_select(d_dph_addr[1:], 16)
				m.d.comb += load_rdata.eq(RCat(Repl(msb(hword_rdata) & d_dph_signed, XLEN - 16), hword_rdata))
			with m.Case():
				byte_rdata = self.hrdata.word_select(d_dph_addr, 8)
				m.d.comb += load_rdata.eq(RCat(Repl(msb(byte_rdata) & d_dph_signed, XLEN - 8), byte_rdata))

		### Stage D/X ###

		opc     = cir[2 :7 ]
		cir_rd  = cir[7 :12]
		funct3  = cir[12:15]
		cir_rs1 = cir[15:20]
		cir_rs2 = cir[20:25]
		funct7  = cir[25:32]

		rs1 = Signal(XLEN)
		rs2 = Signal(XLEN)

		pc = Signal(XLEN, reset=self.reset_vector - 4)

		# ALU, and operand/operation selection

		m.submodules.alu = alu = Hazard2ALU()

		aluop_r_i = Signal(alu.op.shape())
		with m.Switch(funct3):
			with m.Case(0b000):
				# Mask funct7 for I-format (!cir[5]), as it's part of the immediate
				m.d.comb += aluop_r_i.eq(Mux(funct7[5] & cir[5], ALUOp.SUB, ALUOp.ADD))
			with m.Case(0b001):
				m.d.comb += aluop_r_i.eq(ALUOp.SLL)
			with m.Case(0b010):
				m.d.comb += aluop_r_i.eq(ALUOp.LT)
			with m.Case(0b011):
				m.d.comb += aluop_r_i.eq(ALUOp.LTU)
			with m.Case(0b100):
				m.d.comb += aluop_r_i.eq(ALUOp.XOR)
			with m.Case(0b101):
				m.d.comb += aluop_r_i.eq(Mux(funct7[5], ALUOp.SRA, ALUOp.SRL))
			with m.Case(0b110):
				m.d.comb += aluop_r_i.eq(ALUOp.OR)
			with m.Case(0b111):
				m.d.comb += aluop_r_i.eq(ALUOp.AND)

		with m.Switch(opc):
			with m.Case(RVOpc.OP):
				m.d.comb += [
					alu.i0.eq(rs1),
					alu.i1.eq(rs2),
					alu.op.eq(aluop_r_i),
				]
			with m.Case(RVOpc.OP_IMM):
				m.d.comb += [
					alu.i0.eq(rs1),
					alu.i1.eq(imm_i(cir)),
					alu.op.eq(aluop_r_i),
				]
			with m.Case(RVOpc.JAL):
				m.d.comb += [
					alu.i0.eq(pc),
					alu.i1.eq(0),
					alu.op.eq(ALUOp.ADD)
				]
			with m.Case(RVOpc.JALR):
				m.d.comb += [
					alu.i0.eq(pc),
					alu.i1.eq(0),
					alu.op.eq(ALUOp.ADD)
				]
			with m.Case(RVOpc.BRANCH):
				m.d.comb += [
					alu.i0.eq(rs1),
					alu.i1.eq(rs2),
					alu.op.eq(Mux(funct3 & 0x6 == 0x0, ALUOp.SUB,
						Mux(funct3 & 0x6 == 0x4, ALUOp.LT, ALUOp.LTU)))
				]
			with m.Case(RVOpc.LUI):
				m.d.comb += [
					alu.i0.eq(0),
					alu.i1.eq(imm_u(cir)),
					alu.op.eq(ALUOp.ADD)
				]
			with m.Case(RVOpc.AUIPC):
				m.d.comb += [
					alu.i0.eq(pc),
					alu.i1.eq(imm_u(cir)),
					alu.op.eq(ALUOp.ADD),
					alu.take4.eq(True)
				]

		# AGU

		# Don't assert bus request during reset, it's a rude thing to do. Other than
		# that we have the pedal to the metal all the time.
		bus_available = Signal()
		m.d.sync += bus_available.eq(1)
		m.d.comb += self.htrans.eq(RCat(bus_available, C(0, 1)))

		agu_next_addr = Signal(XLEN)

		access_is_load = cir_valid & ~d_dph_active & (opc == RVOpc.LOAD)
		access_is_store = cir_valid & ~d_dph_active & (opc == RVOpc.STORE)
		access_is_loadstore = access_is_load | access_is_store

		take_branch = cir_valid & ~d_dph_active & (opc == RVOpc.BRANCH) & (alu.cmp != funct3[0])
		take_jal = cir_valid & ~d_dph_active & (opc == RVOpc.JAL)
		take_jalr = cir_valid & ~d_dph_active & (opc == RVOpc.JALR)

		with m.If(access_is_load):
			m.d.comb += agu_next_addr.eq(rs1 + imm_i(cir))
		with m.Elif(access_is_store):
			m.d.comb += agu_next_addr.eq(rs1 + imm_s(cir))
		with m.Elif(take_branch):
			m.d.comb += agu_next_addr.eq(pc - 4 + imm_b(cir))
		with m.Elif(take_jal):
			m.d.comb += agu_next_addr.eq(pc - 4 + imm_j(cir))
		with m.Elif(take_jalr):
			m.d.comb += agu_next_addr.eq((rs1 + imm_i(cir)) & -2)
		with m.Else():
			m.d.comb += agu_next_addr.eq(pc + 4)

		# Generate address-phase request
		m.d.comb += self.haddr.eq(agu_next_addr)
		with m.If(access_is_loadstore):
			m.d.comb += [
				self.hwrite.eq(access_is_store),
				self.hsize.eq(RCat(C(0, 1), funct3[:2]))
			]
		with m.Else():
			m.d.comb += [
				self.hsize.eq(2)
			]

		# Update PC and track bus transfer status
		with m.If(bus_available & self.hready):
			with m.If(~access_is_loadstore):
				# Force PC alignment, since we don't support traps
				m.d.sync += pc.eq(agu_next_addr & -4)
			m.d.sync += [
				i_dph_active.eq(self.htrans[1] & ~access_is_loadstore),
				# Note d_dph_active term stops the CIR from being marked as invalid on
				# second cycle of a load/store dphase, since it's not consumed during this
				# time (it is the CIR of the *next* instruction)
				cir_valid.eq((i_dph_active | d_dph_active) & ~(take_branch | take_jal | take_jalr))
			]
			m.d.sync += [
				d_dph_active.eq(self.htrans[1] & access_is_loadstore),
				d_dph_addr.eq(self.haddr[:2]),
				d_dph_size.eq(self.hsize[:2]),
				d_dph_write.eq(self.hwrite),
				d_dph_signed.eq(~funct3[2])
			]


		# Store data shifter
		# Unaligned stores behave correctly as long as you don't do them

		with m.Switch(d_dph_addr):
			with m.Case(0):
				m.d.comb += self.hwdata.eq(rs2)
			with m.Case(1):
				m.d.comb += self.hwdata.eq(RCat(rs2[16:], rs2[:8], rs2[:8]))
			with m.Case(2):
				m.d.comb += self.hwdata.eq(RCat(rs2[:16], rs2[:16]))
			with m.Case(3):
				m.d.comb += self.hwdata.eq(RCat(rs2[:8], rs2[:24]))

		# Register file

		m.submodules.regfile = regfile = Hazard2Regfile()
		m.d.comb += [
			# During load/store, the CIR is updated during cycle n, and the register
			# file is read for next instruction on cycle n + 1, so delay addr using CIR.
			regfile.raddr1.eq(Mux(d_dph_active, cir_rs1, self.hrdata[15:20])),
			regfile.raddr2.eq(Mux(d_dph_active, cir_rs2, self.hrdata[20:25])),
			regfile.ren.eq(~(access_is_loadstore | stall)),
			rs1.eq(regfile.rdata1),
			rs2.eq(regfile.rdata2)
		]

		reg_write_alu = cir_valid & ~d_dph_active & ~stall & (
			(opc == RVOpc.OP) | (opc == RVOpc.OP_IMM) | (opc == RVOpc.JAL) |
			(opc == RVOpc.JALR) | (opc == RVOpc.LUI) | (opc == RVOpc.AUIPC))
		reg_write_load = d_dph_active & ~(stall | d_dph_write)
		load_rd = Signal(cir_rd.shape())
		with m.If(~stall):
			m.d.sync += load_rd.eq(cir_rd)
		m.d.comb += [
			regfile.waddr.eq(Mux(reg_write_load, load_rd, cir_rd)),
			regfile.wdata.eq(Mux(reg_write_load, load_rdata, alu.o)),
			regfile.wen.eq(reg_write_load | reg_write_alu)
		]

		return m
