#!/usr/bin/env python3

import sys
import os.path
sys.path.append(os.path.abspath(os.path.dirname(sys.argv[0]) + "/../hdl"))

import argparse
from nmigen import *
from nmigen.sim import *
from anklesoc import AnkleSoC


def anyint(x):
	return int(x, 0)

class SPIFlash:
	def __init__(self, image):
		if image is None:
			self.image = bytes()
		else:
			self.image = image
		self.addr_bit_count = 0
		self.data_bit_count = 0
		self.read_addr = 0
		self.prev_sck = 0
		self.prev_miso = 0
	def step(self, cs, sck, mosi):
		miso = self.prev_miso
		sck_edge = sck and not self.prev_sck
		self.prev_sck = sck
		if cs:
			self.addr_bit_count = 0
			self.data_bit_count = 0
			self.read_addr = 0
			self.prev_sck = sck
			miso = 0
		elif sck_edge:
			if self.addr_bit_count < 32:
				self.addr_bit_count += 1
				self.read_addr = (self.read_addr << 1) | mosi
			if self.addr_bit_count == 32:
				# Ignore command bits, assume it's a 03h serial read
				self.read_addr &= (1 << 24) - 1
				if self.read_addr < len(self.image):
					miso = (self.image[self.read_addr] >> (7 - self.data_bit_count)) & 1
				self.data_bit_count += 1
				if self.data_bit_count == 8:
					self.data_bit_count = 0
					self.read_addr += 1
		self.prev_miso = miso
		return miso

def main(argv):
	parser = argparse.ArgumentParser()
	parser.add_argument("--flashload", "-f", help="Optional binary file to preload flash")
	parser.add_argument("--ramload", "-r", help="Optional binary file to preload RAM")
	parser.add_argument("--vcdfile", "-v", help="Optional VCD file to write simulation trace")
	parser.add_argument("--memsize", default=4096, type=anyint)
	parser.add_argument("--cycles", default=int(1e4), type=anyint)
	parser.add_argument("--resetvector", type=anyint)

	args = parser.parse_args(argv)
	flash_img = None
	if args.flashload is not None:
		flash_img = open(args.flashload, "rb").read()
	ram_img = None
	if args.ramload is not None:
		ram_img = open(args.ramload, "rb").read()

	dut = AnkleSoC(args.memsize, ram_init=ram_img, cpu_reset_vector=args.resetvector)
	flash = SPIFlash(flash_img)

	def process():
		for i in range(args.cycles):
			yield dut.flash.miso.eq(flash.step(
				(yield dut.flash.cs), (yield dut.flash.clk), (yield dut.flash.mosi)
			))
			yield


	sim = Simulator(dut)
	sim.add_sync_process(process)
	sim.add_clock(1e-6)
	if args.vcdfile is not None:
		with sim.write_vcd(args.vcdfile):
			sim.run()
	else:
		sim.run()

if __name__ == "__main__":
	main(sys.argv[1:])
