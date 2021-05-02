#!/usr/bin/env python3

# Synthesise AnkleSoC on iCEStick, with some given RAM size and RAM init binary.

import sys
import os.path
sys.path.append(os.path.abspath(os.path.dirname(sys.argv[0]) + "/../hdl"))

import argparse
from nmigen import *
from nmigen.sim import *
from nmigen_boards.icestick import ICEStickPlatform
from anklesoc import AnkleSoC


class FPGATop(Elaboratable):
	"""
	Shim class to hide platform capabilities/requests from SoC implementation

	maybe this should be called a Shoe, since it encloses the SoC
	"""
	def __init__(self, soc):
		self._soc = soc

	def elaborate(self, platform):
		m = Module()
		m.submodules.soc = soc = self._soc
		leds = Cat(platform.request("led", i) for i in range(5))
		m.d.comb += leds.eq(soc.leds)
		return m


def anyint(x):
	return int(x, 0)

def main(argv):
	parser = argparse.ArgumentParser()
	parser.add_argument("binfile"),
	parser.add_argument("--memsize", default=1 << 10, type = anyint)
	parser.add_argument("--program", "-p", action="store_true")

	args = parser.parse_args(argv)
	mem_init = open(args.binfile, "rb").read()
	top = FPGATop(AnkleSoC(ram_size_bytes=args.memsize, ram_init=mem_init))
	ICEStickPlatform().build(top, program=args.program)

if __name__ == "__main__":
	main(sys.argv[1:])
