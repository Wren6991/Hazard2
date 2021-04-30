#!/usr/bin/env python3

from nmigen.back import verilog
from hazard2 import Hazard2CPU

cpu = Hazard2CPU()
with open("hazard2_cpu.v", "w") as f:
	f.write(verilog.convert(cpu, name="hazard2_cpu", ports=[cpu.htrans, cpu.hwrite, cpu.hsize,  cpu.haddr, cpu.hwdata, cpu.hrdata, cpu.hready]))

