import sys

from migen.fhdl import *
from migen import *

from hexastorm import board

class Pincheck(Module):
    def __init__(self, plat):
        '''connects photodiode pin to raspberry pi to read out status
        '''
        ph_diode = plat.request("photodiode")
        gpio14 = plat.request("gpio14")
        laser0 = plat.request("laser0")
        self.sync += laser0.eq(1)
        self.sync += gpio14.eq(ph_diode)

# build
plat = board.Platform()
pincheck = Pincheck(plat)
build_name = 'pincheck'
plat.build(pincheck, build_name = build_name)
# upload
plat.upload(build_name)
# remove build dir
plat.removebuild()