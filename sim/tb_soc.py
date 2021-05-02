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

def main(argv):
	parser = argparse.ArgumentParser()
	parser.add_argument("binfile"),
	parser.add_argument("vcdfile", nargs="?")
	parser.add_argument("--memsize", default=1 << 10, type = anyint)
	parser.add_argument("--cycles", default=int(1e4), type = anyint)

	args = parser.parse_args(argv)
	mem_init = open(args.binfile, "rb").read()

	def process():
		for i in range(args.cycles):
			yield

	dut = AnkleSoC(args.memsize, mem_init)
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
