from nmigen import *
from nmigen.utils import log2_int
from functools import reduce

from hazard2 import Hazard2CPU

# TODO this could probably be a Record once I figure out the direction stuff
class AHBLPort:
	"""AHB-Lite port interface (or a subset of one -- no bursts, no error
	signalling, no protection attributes, no locking).

	Using convention that master and slave ports are identical, so decoder masks
	HTRANS rather than driving an HSEL, and HREADY + HREADYOUT (hready_resp) are
	both present on all ports, with hready_resp->hready tied at top of fabric
	"""
	def __init__(self, awidth, dwidth):
		self.htrans      = Signal(2)
		self.hwrite      = Signal()
		self.hsize       = Signal(3)
		self.haddr       = Signal(awidth)
		self.hwdata      = Signal(dwidth)
		self.hrdata      = Signal(dwidth)
		self.hready      = Signal()
		self.hready_resp = Signal()

	def connect_to_upstream(self, parent, other):
		parent.d.comb += [
			self .htrans      .eq(other.htrans     ),
			self .hwrite      .eq(other.hwrite     ),
			self .hsize       .eq(other.hsize      ),
			self .haddr       .eq(other.haddr      ),
			self .hwdata      .eq(other.hwdata     ),
			self .hready      .eq(other.hready     ),
			other.hrdata      .eq(self .hrdata     ),
			other.hready_resp .eq(self .hready_resp)
		]

	def connect_to_downstream(self, parent, other):
		other.connect_to_upstream(parent, self)


class AHBLSplitter(Elaboratable):
	"""
	Connect one upstream AHB-Lite port to multiple downstream ports, routing
	requests and responses appropriately.

	`ports` is a list of (base, mask) tuples that defines the downstream address
	map. One downstream port is generated for each entry in `ports`.
	"""
	def __init__(self, awidth, dwidth, ports):
		self._dwidth = dwidth
		self.up = AHBLPort(awidth, dwidth)
		self.down = [AHBLPort(awidth, dwidth) for p in ports]
		self._addr_map = ports

	def elaborate(self, platform):
		m = Module()

		decode_aph = Cat(self.up.haddr & mask == base for base, mask in self._addr_map)
		decode_aph_masked = decode_aph & Repl(self.up.htrans[1], decode_aph.shape().width)
		decode_dph = Signal(decode_aph.shape())

		with m.If(self.up.hready):
			m.d.sync += decode_dph.eq(decode_aph_masked)

		for i, p in enumerate(self.down):
			m.d.comb += [
				# Downstream address-phase request
				p.htrans.eq(self.up.htrans & Repl(decode_aph[i], 2)),
				p.hwrite.eq(self.up.hwrite),
				p.hsize.eq(self.up.hsize),
				p.haddr.eq(self.up.haddr),
				# Downstream data-phase fanout
				p.hwdata.eq(self.up.hwdata),
				p.hready.eq(self.up.hready)
			]
		# Upstream data-phase muxing
		m.d.comb += self.up.hready_resp.eq(~decode_dph.any() | (
			decode_dph & Cat(p.hready_resp for p in self.down)).any())
		m.d.comb += self.up.hrdata.eq(reduce(lambda a, b: a | b,
			(p.hrdata & Repl(decode_dph[i], self._dwidth) for i, p in enumerate(self.down))))

		return m


class AHBLBlinky(Elaboratable):
	"""
	Blink LEDs over AHB-Lite. To minimise hardware footprint, the junior haddr
	bits are used as write data, and hwdata is ignored. E.g.:

	(void)*(volatile uint8_t*)(BLINKY_BASE + 0x5);

	will illuminate LEDS 0 and 2.
	"""
	def __init__(self, awidth, dwidth, n_leds):
		assert(n_leds <= awidth)
		self.bus = AHBLPort(awidth, dwidth)
		self.leds = Signal(n_leds)

	def elaborate(self, platform):
		m = Module()
		m.d.comb += [
			self.bus.hready_resp.eq(1),
			self.bus.hrdata.eq(0)
		]
		with m.If(self.bus.htrans[1] & self.bus.hready):
			m.d.sync += self.leds.eq(self.bus.haddr)
		return m


class AHBLSRAM(Elaboratable):
	"""
	An SRAM wrapped in an AHB-Lite slave interface. Write-to-read collision has a
	one cycle penalty, otherwise all accesses are zero-wait-state.

	init is a bytes or bytearray object. If the SRAM is multiple bytes wide, the
	bytes are packed in little-endian order.
	"""
	def __init__(self, awidth, dwidth, depth, init=None):
		self._awidth = awidth
		self._dwidth = dwidth
		self._depth = depth

		if init is None:
			init_words = None
		else:
			init_words = []
			dwidth_bytes = dwidth // 8
			assert(type(init) is bytes or type(init) is bytearray)
			if len(init) % dwidth_bytes != 0:
				init = init + bytes(dwidth_bytes - len(init) % dwidth_bytes)
			assert(len(init) <= depth * dwidth_bytes)
			for i in range(0, len(init), dwidth_bytes):
				accum = 0
				# Little-endian:
				for j in reversed(range(dwidth_bytes)):
					accum = accum << 8 | init[i + j]
				init_words.append(accum)

		self.bus = AHBLPort(awidth, dwidth)
		self.mem = Memory(width=dwidth, depth=depth, init=init_words)


	def elaborate(self, platform):
		m = Module()
		m.submodules.rport = rport = self.mem.read_port(transparent=False)
		m.submodules.wport = wport = self.mem.write_port(granularity=8)

		bytesel_width = log2_int(self._dwidth // 8)
		addr_aph = self.bus.haddr[bytesel_width : bytesel_width + log2_int(self._depth)]
		read_aph  = self.bus.htrans[1] & self.bus.hready & ~self.bus.hwrite
		write_aph = self.bus.htrans[1] & self.bus.hready &  self.bus.hwrite

		addr_dph = Signal(addr_aph.shape())
		delayed_read_dph = Signal()
		write_dph = Signal()
		write_byte_en = Signal(self._dwidth // 8)

		# AHBL state transitions and address-phase capture
		with m.If(self.bus.hready):
			with m.If(write_aph | (read_aph & write_dph)):
				m.d.sync += addr_dph.eq(addr_aph)
			m.d.sync += [
				write_dph.eq(write_aph),
				delayed_read_dph.eq(read_aph & write_dph),
				write_byte_en.eq((1 << (1 << self.bus.hsize)) - 1 << addr_aph[:bytesel_width])
			]

		# SRAM control decode
		sram_addr = Mux(write_dph | delayed_read_dph, addr_dph, addr_aph)
		m.d.comb += [
			rport.addr.eq(sram_addr),
			rport.en.eq(read_aph | delayed_read_dph),
			wport.addr.eq(sram_addr),
			wport.en.eq(write_byte_en & Repl(write_dph, self._dwidth // 8)),
			wport.data.eq(self.bus.hwdata)
		]

		# Bus response
		m.d.comb += [
			self.bus.hrdata.eq(rport.data),
			self.bus.hready_resp.eq(~delayed_read_dph)
		]

		return m


class AnkleSoC(Elaboratable):
	"""
	AnkleSoC -- the smallest useful sock
	"""
	def __init__(self, ram_size_bytes=1024, ram_init=None):
		self._ram_size_bytes = ram_size_bytes
		self._ram_init = ram_init
		self._n_leds = 5

		self.leds = Signal(self._n_leds)

	def elaborate(self, platform):
		m = Module()
		awidth = 32
		dwidth = 32
		ram_depth = self._ram_size_bytes // (dwidth // 8)
		m.submodules.cpu = cpu = Hazard2CPU(reset_vector=0x0)
		m.submodules.ram = ram = AHBLSRAM(awidth=awidth, dwidth=dwidth, depth=ram_depth, init=self._ram_init)
		m.submodules.led = led = AHBLBlinky(awidth=awidth, dwidth=dwidth, n_leds = self._n_leds)
		m.submodules.splitter = splitter = AHBLSplitter(awidth=awidth, dwidth=dwidth, ports=(
			(0x0000_0000, 0x8000_0000), # 2 GB RAM segment
			(0x8000_0000, 0x8000_0000), # 2 GB IO segment
		))
		splitter.down[0].connect_to_downstream(m, ram.bus)
		splitter.down[1].connect_to_downstream(m, led.bus)

		# Upstream splitter port needs connecting manually because the processor
		# port doesn't use our AHBLPort class (since it's intended for standalone use)
		m.d.comb += [
			splitter.up.htrans.eq(cpu.htrans),
			splitter.up.hwrite.eq(cpu.hwrite),
			splitter.up.hsize.eq(cpu.hsize),
			splitter.up.haddr.eq(cpu.haddr),
			splitter.up.hwdata.eq(cpu.hwdata),
			cpu.hrdata.eq(splitter.up.hrdata),
			# Tie hready_resp -> hready at top of fabric
			cpu.hready.eq(splitter.up.hready_resp),
			splitter.up.hready.eq(splitter.up.hready_resp)
		]

		m.d.comb += self.leds.eq(led.leds)

		return m
