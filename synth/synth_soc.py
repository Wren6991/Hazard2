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
	parser.add_argument("binfile"),
	parser.add_argument("--memsize", default=4096, type=anyint)
	parser.add_argument("--program", "-p", action="store_true")

	args = parser.parse_args(argv)
	mem_init = open(args.binfile, "rb").read()
	top = AnkleSoC(ram_size_bytes=args.memsize, ram_init=mem_init)
	ICEStickPlatform().build(top, program=args.program)

if __name__ == "__main__":
	main(sys.argv[1:])
