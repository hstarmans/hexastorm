"""
    Test for spinning polygon motor
"""
import sys

from migen.fhdl import *
from migen import *

from hexastorm import board

class Polygon(Module):
    def __init__(self, poly_en, poly_pwm, poly_freq):
        '''poly freq is in Hz
        '''
        maxperiod = int(100e6/poly_freq)
        counter = Signal(max=maxperiod)
        self.sync += poly_en.eq(0)
        self.sync += If(counter == 0,
                poly_pwm.eq(~poly_pwm),
                counter.eq(maxperiod-1)).Else(
                counter.eq(counter - 1)
                )

# set up single channel led
plat = board.Platform()
poly_en = plat.request("poly_en")
poly_pwm = plat.request("poly_pwm")

polygon = Polygon(poly_en, poly_pwm, 20)
build_name = 'spin'
# build
plat.build(polygon, build_name = build_name)
# upload
plat.upload(build_name)
# remove build dir
plat.removebuild()