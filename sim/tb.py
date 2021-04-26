#!/usr/bin/env python3

import sys
import os.path
sys.path.append(os.path.abspath(os.path.dirname(sys.argv[0]) + "/../hdl"))

import argparse
from nmigen import *
from nmigen.sim import *
from hazard2 import Hazard2CPU

def anyint(x):
	return int(x, 0)

def main(argv):
	parser = argparse.ArgumentParser()
	parser.add_argument("binfile")
	parser.add_argument("--memsize", default = 1 << 24, type = anyint)
	parser.add_argument("--cycles", default = int(1e4), type = anyint)
	parser.add_argument("--dump", nargs=2, action="append", type=anyint)
	args = parser.parse_args(argv)
	mem = bytearray(args.memsize)

	bindata = open(args.binfile, "rb").read()
	assert(len(bindata) < args.memsize)
	mem[:len(bindata)] = bindata

	def process():
		yield dut.hready.eq(1)
		yield
		dph_active = False
		dph_write = False
		dph_addr = 0
		dph_size = 0
		for i in range(args.cycles):
			# Apply data-phase write to testbench state
			if dph_active and dph_write:
				wdata = yield dut.hwdata
				if dph_addr >= 0x80000000:
					if dph_addr == 0x80000000:
						sys.stdout.write(chr(wdata & 0xff))
					elif dph_addr == 0x80000008:
						print(f"CPU terminated simulation with exit code {wdata}")
						break
					else:
						raise Exception(f"Unknown IO address {dph_addr:08x}")
				else:
					for i in range(4):
						if i >= (dph_addr & 0x3) and i < ((dph_addr & 0x3) + (1 << dph_size)):
							mem[(dph_addr & -4) + i] = (wdata >> i * 8) & 0xff

			# Progress current address-phase request to data phase
			dph_active = (yield dut.htrans) >> 1
			dph_write = yield dut.hwrite
			dph_addr = yield dut.haddr
			dph_size = yield dut.hsize
			# Perform data-phase read, ready for next simulation cycle
			if dph_active and not dph_write:
				yield dut.hrdata.eq(sum(mem[(dph_addr & -4) + i] << i * 8 for i in range(4)))
			# Step the processor
			yield

		print(f"Ran for {i + 1} cycles")


	dut = Hazard2CPU() # electric boogaloo
	sim = Simulator(dut)
	sim.add_sync_process(process)
	sim.add_clock(1e-6)
	with sim.write_vcd("test.vcd"):
		sim.run()

	for start, end in args.dump or []:
		print(f"Dumping memory from {start:08x} to {end:08x}:")
		for i, addr in enumerate(range(start, end)):
			sep = "\n" if i % 16 == 15 else " "
			sys.stdout.write(f"{mem[addr]:02x}{sep}")
		print("")

if __name__ == "__main__":
	main(sys.argv[1:])
