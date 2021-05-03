#!/usr/bin/env python3

# Synthesise AnkleSoC on iCEStick, with some given RAM size and RAM init binary.

import sys
import os.path
sys.path.append(os.path.abspath(os.path.dirname(sys.argv[0]) + "/../hdl"))

import argparse
from nmigen import *
from nmigen_boards.icestick import ICEStickPlatform
from anklesoc import AnkleSoC

def anyint(x):
	return int(x, 0)

def main(argv):
	parser = argparse.ArgumentParser()
	parser.add_argument("--ramload", "-r", help="Optional binary file to preload RAM")
	parser.add_argument("--memsize", default=4096, type=anyint)
	parser.add_argument("--resetvector", type=anyint)
	parser.add_argument("--program", "-p", action="store_true")

	args = parser.parse_args(argv)
	mem_init = None
	if args.ramload is not None:
		mem_init = open(args.ramload, "rb").read()
	top = AnkleSoC(ram_size_bytes=args.memsize, ram_init=mem_init, cpu_reset_vector=args.resetvector)
	ICEStickPlatform().build(top, program=args.program, synth_opts="-abc2")

if __name__ == "__main__":
	main(sys.argv[1:])
