from nmigen import *
from nmigen.utils import log2_int
from functools import reduce
from hazard2 import Hazard2CPU

# AnkleSoC: Minimal system-on-chip for Hazard2.

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

		decode_aph = Cat((self.up.haddr & mask) == base for base, mask in self._addr_map)
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
			m.d.sync += addr_dph.eq(addr_aph)
			m.d.sync += [
				write_dph.eq(write_aph),
				write_byte_en.eq((1 << (1 << self.bus.hsize)) - 1 << self.bus.haddr[:bytesel_width])
			]
		m.d.sync += delayed_read_dph.eq(read_aph & write_dph)

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


class AHBLFlashXIP(Elaboratable):
	"""
	Tiny flash execute-in-place interface, with direct-mapped read-only cache,
	zero-wait-state read hit. A 1 kB cache is the smallest efficient size on
	iCE40: this uses two BRAMs for the 256x32b data memory, and one BRAM for the
	256x15b tag memory (so only ~2% of the 3 BRAMs are wasted).

	On cache miss, always perform 03h serial read commands with size equal to the
	bus width, and address aligned down to bus width, regardless of read HSIZE.
	SCK runs at half the sync domain clock frequency.
	"""

	SERIAL_READ_CMD = 0x03
	FLASH_ADDR_WIDTH = 24

	def __init__(self, awidth, dwidth, cache_size_bytes=1024, flash_size_bytes=16*1024*1024):
		self._awidth = awidth
		self._dwidth = dwidth

		self._depth = cache_size_bytes // (dwidth // 8)
		self._index_width = log2_int(self._depth)
		self._offset_width = log2_int(dwidth // 8)
		self._tag_width = self.FLASH_ADDR_WIDTH - (self._index_width + self._offset_width)
		# We can use smaller tags/comparisons if the downstream flash is smaller than 16 MB.
		self._tag_mask = (1 << log2_int(flash_size_bytes // cache_size_bytes)) - 1
		self._flash_size_bytes = flash_size_bytes

		self.tmem = Memory(width=self._tag_width + 1, depth=self._depth, init=[0] * self._depth)
		self.dmem = Memory(width=self._dwidth, depth=self._depth)

		self.bus = AHBLPort(awidth, dwidth)
		# Using active-high chip select convention, to match nMigen platform code
		self.pad_cs   = Signal()
		self.pad_sck  = Signal()
		self.pad_mosi = Signal()
		self.pad_miso = Signal()


	def elaborate(self, platform):
		m = Module()

		sreg = Signal(max(self.FLASH_ADDR_WIDTH + 8, self._dwidth))
		shift_ctr = Signal(range(sreg.width))
		m.d.comb += self.pad_mosi.eq(sreg[self.FLASH_ADDR_WIDTH + 8 - 1])

		bus_addr_mask = (1 << log2_int(self._flash_size_bytes)) - (1 << self._offset_width)
		cmd_plus_addr = Cat(
			(self.bus.haddr & bus_addr_mask)[:self.FLASH_ADDR_WIDTH],
			C(self.SERIAL_READ_CMD, 8)
		)

		# Decode bus controls, and capture useful address bits for data phase

		aph_read = self.bus.htrans[1] & self.bus.hready & ~self.bus.hwrite

		aph_offset = Signal(self._offset_width)
		aph_index = Signal(self._index_width)
		aph_tag = Signal(self._tag_width)
		m.d.comb += Cat(aph_offset, aph_index, aph_tag).eq(self.bus.haddr)

		# The tag is only needed on the first dphase cycle, to check for hit, and
		# install the new tag on miss. This means we can get away with snooping it
		# from the SPI shift register. The index is required post-shift, for data
		# writeback, so needs to be registered separately.
		dph_tag = sreg[self.FLASH_ADDR_WIDTH - self._tag_width:self.FLASH_ADDR_WIDTH]
		dph_index = Signal(self._index_width)
		with m.If(aph_read):
			m.d.sync += [
				sreg.eq(cmd_plus_addr),
				dph_index.eq(aph_index)
			]

		# Instantiate and hookup memory ports

		m.submodules.twport = twport = self.tmem.write_port()
		m.submodules.trport = trport = self.tmem.read_port(transparent=False)
		m.submodules.dwport = dwport = self.dmem.write_port()
		m.submodules.drport = drport = self.dmem.read_port(transparent=False)

		tmem_rvalid = Signal()
		tmem_rtag = Signal(self._tag_width)
		sreg_endian_swap = Cat(
			sreg.word_select(i, 8) for i in reversed(range(self._dwidth // 8))
		)

		# dmem may be read twice: once when entering HIT/MISS state to speculatively
		# pull out the hit data, and once after data writeback (on miss) to pull out
		# the fresh read data onto hrdata. Note the second read costs us a cycle on
		# miss, and costs 8 muxes to get two different addresses into the read port
		# (assuming 1kB cache), but it saves 32 muxes (!) on hrdata, so worth it.
		dmem_read_in_dph = Signal()

		m.d.comb += [
			trport.addr.eq(aph_index),
			Cat(tmem_rtag, tmem_rvalid).eq(trport.data),
			twport.addr.eq(dph_index),
			twport.data.eq(Cat(dph_tag & self._tag_mask, C(1, 1))),

			drport.addr.eq(Mux(dmem_read_in_dph, dph_index, aph_index)),
			self.bus.hrdata.eq(drport.data),
			dwport.addr.eq(dph_index),
			dwport.data.eq(sreg_endian_swap),
		]

		cache_hit = Signal()
		m.d.comb += cache_hit.eq(tmem_rvalid & ((tmem_rtag & self._tag_mask) == dph_tag))

		with m.FSM() as fsm:
			with m.State("IDLE"):
				with m.If(aph_read):
					m.next = "HIT/MISS"
			with m.State("HIT/MISS"):
				with m.If(cache_hit):
					with m.If(aph_read):
						m.next = "HIT/MISS"
					with m.Else():
						m.next = "IDLE"
				with m.Else():
					m.next = "CMD/ADDR"
					m.d.sync += [
						self.pad_cs.eq(1),
						shift_ctr.eq(self.FLASH_ADDR_WIDTH + 8 - 1)
					]
			with m.State("CMD/ADDR"):
				m.d.sync += self.pad_sck.eq(~self.pad_sck)
				with m.If(self.pad_sck):
					m.d.sync += [
						sreg.eq(sreg << 1),
						shift_ctr.eq(shift_ctr - 1)
					]
				with m.If(self.pad_sck & ~shift_ctr.any()):
					m.next = "DATA"
					m.d.sync += shift_ctr.eq(self._dwidth - 1)
			with m.State("DATA"):
				m.d.sync += self.pad_sck.eq(~self.pad_sck)
				with m.If(self.pad_sck):
					m.d.sync += shift_ctr.eq(shift_ctr - 1)
				with m.Else():
					m.d.sync += sreg.eq((sreg << 1) | self.pad_miso)
				with m.If(self.pad_sck & ~shift_ctr.any()):
					m.next = "BACKPORCH"
			with m.State("BACKPORCH"):
				m.d.sync += self.pad_cs.eq(0)
				# dmem readback happens this cycle, so the aph read can't take place until
				# the next cycle. This means we must return to the idle state so aph read
				# can take place there, we can't skip straight to HIT/MISS of next access.
				m.next = "IDLE"
			m.d.comb += [
				self.bus.hready_resp.eq(fsm.ongoing("IDLE") |
						(fsm.ongoing("HIT/MISS") & cache_hit)),
				twport.en.eq(fsm.ongoing("HIT/MISS") & ~cache_hit),
				trport.en.eq(aph_read),
				dwport.en.eq(fsm.ongoing("DATA") & self.pad_sck & ~shift_ctr.any()),
				drport.en.eq(aph_read | fsm.ongoing("BACKPORCH")),
				dmem_read_in_dph.eq(fsm.ongoing("BACKPORCH"))
			]

		return m


class AnkleSoC(Elaboratable):
	"""
	AnkleSoC -- the smallest useful sock
	"""
	def __init__(self, ram_size_bytes=4096, ram_init=None, n_leds=5, cpu_reset_vector=None):
		self._ram_size_bytes = ram_size_bytes
		self._ram_init = ram_init
		self._n_leds = n_leds
		if cpu_reset_vector is None:
			self._cpu_reset_vector = 0x0040_0000
		else:
			self._cpu_reset_vector = cpu_reset_vector
		# Testbench-only signals:
		self.flash = None
		self.leds = None
		self.uart = None

	def elaborate(self, platform):
		m = Module()
		awidth = 32
		dwidth = 32
		ram_depth = self._ram_size_bytes // (dwidth // 8)
		m.submodules.cpu = cpu = Hazard2CPU(reset_vector=self._cpu_reset_vector)
		m.submodules.ram = ram = AHBLSRAM(awidth, dwidth, depth=ram_depth, init=self._ram_init)
		m.submodules.led = led = AHBLBlinky(awidth, dwidth, n_leds=self._n_leds)
		m.submodules.utx = utx = AHBLBlinky(awidth, dwidth, n_leds=1)
		m.submodules.xip = xip = AHBLFlashXIP(awidth, dwidth, flash_size_bytes=0x40_0000)
		# Addresses are mapped so that major decode is one-hot on the MSBs.
		m.submodules.splitter = splitter = AHBLSplitter(awidth, dwidth, ports=(
			(0x0040_0000, 0x0040_0000), # 4 MB flash segment
			(0x0080_0000, 0x0080_0000), # 4 MB RAM segment
			(0x0100_0000, 0x0100_1000), # 4 MB IO segment, first register
			(0x0100_1000, 0x0100_1000), # 4 MB IO segment, second register
		))
		splitter.down[0].connect_to_downstream(m, xip.bus)
		splitter.down[1].connect_to_downstream(m, ram.bus)
		splitter.down[2].connect_to_downstream(m, led.bus)
		splitter.down[3].connect_to_downstream(m, utx.bus)

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

		# IO hookup
		if platform is None:
			self.leds = leds = Signal(self._n_leds)
			self.flash = flash = Record((("mosi", 1), ("miso", 1), ("clk", 1), ("cs", 1)))
			self.uart = uart = Record((("tx", 1),))
		else:
			leds = Cat(platform.request("led", i) for i in range(self._n_leds))
			flash = platform.request("spi_flash_1x")
			uart = platform.request("uart")
		m.d.comb += [
			leds.eq(led.leds),
			uart.tx.eq(utx.leds),
			flash.cs.eq(xip.pad_cs),
			flash.clk.eq(xip.pad_sck),
			flash.mosi.eq(xip.pad_mosi),
			xip.pad_miso.eq(flash.miso)
		]

		return m
