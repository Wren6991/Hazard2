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

def imm_i(instr):
	return Cat(instr[20:], Repl(instr[-1], 20))

def imm_s(instr):
	return Cat(instr[7:12], instr[25:], Repl(instr[-1], 20))

def imm_b(instr):
	return Cat(C(0, 1), instr[8:12],  instr[25:31], instr[7], Repl(instr[-1], 20))

def imm_u(instr):
	return Cat(C(0, 12), instr[12:])

def imm_j(instr):
	return Cat(C(0, 1), instr[21:31], instr[20], instr[12:20], Repl(instr[-1], 12))


class Hazard2Shifter(Elaboratable):
	def __init__(self):
		self.i     = Signal(XLEN)
		self.shamt = Signal(range(XLEN))
		self.right = Signal()
		self.arith = Signal()
		self.o     = Signal(XLEN)

	def elaborate(self, platform):
		m = Module()

		accum = Signal(XLEN, name="shift_pre_reverse")
		m.d.comb += accum.eq(Mux(self.right, self.i, self.i[::-1]))
		for i in range(self.shamt.width):
			accum_next = Signal(XLEN, name=f"shift_accum{i}")
			m.d.comb += accum_next.eq(Mux(self.shamt[i],
				Cat(accum[1 << i:], Repl(accum[-1] & self.arith, 1 << i)),
				accum
			))
			accum = accum_next
		m.d.comb += self.o.eq(Mux(self.right, accum, accum[::-1]))

		return m


class Hazard2ALU(Elaboratable):
	def __init__(self):
		self.i0    = Signal(XLEN)
		self.i1    = Signal(XLEN)
		self.shift = Signal(XLEN)
		self.op    = Signal(Shape.cast(ALUOp))
		self.take4 = Signal()
		self.cmp   = Signal()
		self.o     = Signal(XLEN)

	def elaborate(self, platform):
		m = Module()

		# Add/subtract i0 and i1, then subtract 4 if take4 is true. Use of 3-input adder
		# encourages tools to implement as carry-save.
		adder = sum((
			self.i0,
			self.i1 ^ Repl(self.op != ALUOp.ADD, XLEN),
			Cat(self.op != ALUOp.ADD, C(0, 1), Repl(self.take4, XLEN - 2))
		))[:XLEN]

		less_than = Mux(self.i0[-1] == self.i1[-1], adder[-1],
			Mux(self.op == ALUOp.LTU, self.i1[-1], self.i0[-1])
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
				m.d.comb += self.o.eq(self.shift)
			with m.Case(ALUOp.SRA):
				m.d.comb += self.o.eq(self.shift)
			with m.Case(ALUOp.SLL):
				m.d.comb += self.o.eq(self.shift)
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
				m.d.comb += load_rdata.eq(Cat(hword_rdata, Repl(hword_rdata[-1] & d_dph_signed, XLEN - 16)))
			with m.Case():
				byte_rdata = self.hrdata.word_select(d_dph_addr, 8)
				m.d.comb += load_rdata.eq(Cat(byte_rdata, Repl(byte_rdata[-1] & d_dph_signed, XLEN - 8)))

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
		m.submodules.shifter = shifter = Hazard2Shifter()

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

		# Shifter is used for both shift instructions and store-data alignment.
		m.d.comb += [
			shifter.i.eq(rs1),
			shifter.shamt.eq(Mux(
				d_dph_active, d_dph_addr << 3, Mux( # Store (CIR invalid at this point)
				~opc[3],      cir_rs2,              # RVOpc.OP_IMM  0b00_100
				              rs2[:5]               # RVOpc.OP      0b01_100
			))),
			shifter.right.eq(~d_dph_active & (alu.op != ALUOp.SLL)),
			shifter.arith.eq(~d_dph_active & (alu.op == ALUOp.SRA)),
			alu.shift.eq(shifter.o),
			self.hwdata.eq(shifter.o)
		]

		# AGU

		# Don't assert bus request during reset, it's a rude thing to do. Other than
		# that we have the pedal to the metal all the time.
		bus_available = Signal()
		m.d.sync += bus_available.eq(1)
		m.d.comb += self.htrans.eq(bus_available << 1)

		agu_next_addr = Signal(XLEN)

		access_is_load = cir_valid & ~d_dph_active & (opc == RVOpc.LOAD)
		access_is_store = cir_valid & ~d_dph_active & (opc == RVOpc.STORE)
		access_is_loadstore = access_is_load | access_is_store

		take_branch = cir_valid & ~d_dph_active & (opc == RVOpc.BRANCH) & (alu.cmp != funct3[0])
		take_jal = cir_valid & ~d_dph_active & (opc == RVOpc.JAL)
		take_jalr = cir_valid & ~d_dph_active & (opc == RVOpc.JALR)

		agu_op0 = Signal(XLEN)
		agu_op1 = Signal(XLEN)
		agu_offs = Signal(XLEN)

		with m.If(access_is_load):
			m.d.comb += [agu_op0.eq(rs1), agu_op1.eq(imm_i(cir))]
		with m.Elif(access_is_store):
			m.d.comb += [agu_op0.eq(rs1), agu_op1.eq(imm_s(cir))]
		with m.Elif(take_branch):
			m.d.comb += [agu_op0.eq(pc), agu_op1.eq(imm_b(cir))]
		with m.Elif(take_jal):
			m.d.comb += [agu_op0.eq(pc), agu_op1.eq(imm_j(cir))]
		with m.Elif(take_jalr):
			m.d.comb += [agu_op0.eq(rs1), agu_op1.eq(imm_i(cir))]
		with m.Else():
			m.d.comb += [agu_op0.eq(pc), agu_op1.eq(0)]

		# Offset of +/-4 applied via third adder input (which tools will likely implement as carry-save)
		m.d.comb += agu_offs.eq(Cat(
			C(0, 2), 
			((take_branch | take_jal) & ~access_is_loadstore) | ~(access_is_loadstore | take_jalr),
			Repl((take_branch | take_jal) & ~access_is_loadstore, 29)
		))
		m.d.comb += agu_next_addr.eq(agu_op0 + agu_op1 + agu_offs)

		# Generate address-phase request
		m.d.comb += self.haddr.eq(agu_next_addr)
		with m.If(access_is_loadstore):
			m.d.comb += [
				self.hwrite.eq(access_is_store),
				self.hsize.eq(funct3[:2])
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

		# Register file

		m.submodules.regfile = regfile = Hazard2Regfile()
		m.d.comb += [
			# During load/store, the CIR is updated during aphase, and the register
			# file is read for next instruction during dphase, so delay regaddr using CIR.
			# Also during store aphase we need to read rs2 through the rs1 port in time
			# for the dphase, so we can shift it.
			regfile.raddr1.eq(Mux(access_is_store, cir_rs2,
				Mux(d_dph_active, cir_rs1, self.hrdata[15:20]))),
			regfile.raddr2.eq(Mux(d_dph_active, cir_rs2, self.hrdata[20:25])),
			regfile.ren.eq(~stall),
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
