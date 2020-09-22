import sys

from migen.fhdl import *
from migen import *

from hexastorm import board

class Blinky(Module):
    def __init__(self, led1, led2, maxperiod):
        counter = Signal(max=int(maxperiod))
        self.sync += If(counter == 0,
                led1.eq(~led1),
                led2.eq(~led2),
                counter.eq(int(maxperiod))).Else(
                counter.eq(counter - 1)
                )

# set up single channel led
plat = board.Platform()
led2 = plat.request("led2")
led3 = plat.request("led3")
my_Blinky = Blinky(led2, led3, int(100E6))

# build
build_name = 'blinky'
plat.build(freq=100, core=my_Blinky, build_name = build_name)
# upload
plat.upload(build_name)
# remove build dir
plat.removebuild()