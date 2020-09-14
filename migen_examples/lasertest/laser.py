import sys

from migen.fhdl import *
from migen import *

from hexastorm import board

class Blinky(Module):
    def __init__(self, laser, laser_freq):
        '''poly freq is in Hz

        '''
        maxperiod = int(100e6/laser_freq)
        counter = Signal(max=maxperiod)
        self.sync += If(counter == 0,
                laser.eq(~laser),
                counter.eq(int(maxperiod))).Else(
                counter.eq(counter - 1)
                )

# set up single channel led
plat = board.Platform()
laser = plat.request("laser1")
my_Blinky = Blinky(laser, 10)
# build
plat.build(my_Blinky, build_name = 'laser1')

# # flash
# prog = board.IceStormProgrammer()
# prog.flash(0, "build/top.bin")