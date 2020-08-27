"""
    blinky.py
    Basic blinky example for migen. 
    
    Rik starmans
"""

from migen.fhdl import *
from migen import *
import sys
sys.path.append("..") 
import hexa as board

class Blinky(Module):
    def __init__(self, led1, led2, maxperiod):
        counter = Signal(max=int(maxperiod))
        self.sync += If(counter == 0,
                led1.eq(~led1),
                led2.eq(~led2),
                counter.eq(int(maxperiod))).Else(
                counter.eq(counter - 1)
                )

# print
led1 = Signal()
led2 = Signal()
my_Blinky = Blinky(led1, led2, int(100E6))
print(verilog.convert(my_Blinky, ios={led1, led2}))

# set up single channel led
plat = board.Platform()
led2 = plat.request("led2")
led3 = plat.request("led3")
my_Blinky = Blinky(led2, led3, int(100E6))

# build
plat.build(my_Blinky, build_name = 'blinky')

# # flash
# prog = board.IceStormProgrammer()
# prog.flash(0, "build/top.bin")