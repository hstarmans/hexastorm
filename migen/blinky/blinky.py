"""
    blinky.py
    Basic blinky example for migen. 
    Tested with icezero board
    
    Rik starmans
"""

from migen.fhdl import *
from migen import *
import sys
sys.path.append("..") 
import icezero as board

class Blinky(Module):
    def __init__(self, led, maxperiod):
        counter = Signal(max=maxperiod+1)
        period = Signal(max=maxperiod+1)
        self.comb += period.eq(maxperiod)
        self.sync += If(counter == 0,
                led.eq(~led),
                counter.eq(period)).Else(
                counter.eq(counter - 1)
                )

# print
led = Signal()
my_Blinky = Blinky(led, int(100E6))
print(verilog.convert(my_Blinky, ios={led}))

# set up single channel led
plat = board.Platform()
led = plat.request("user_led")
my_Blinky = Blinky(led, int(100E6))

# build
plat.build(my_Blinky, build_name = 'blinky')

# # flash
# prog = board.IceStormProgrammer()
# prog.flash(0, "build/top.bin")