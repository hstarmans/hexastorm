import sys

from migen.fhdl import *
from migen import *

from hexastorm import board

class Photodiode(Module):
    def __init__(self, laser, poly_en, poly_pwm, poly_freq, ph_diode, pin14):
        '''poly freq is in Hz

        '''
        maxperiod = int(100e6/poly_freq)
        counter = Signal(max=maxperiod)
         # Photodiode falling edge detector
        photodiode_d = Signal()
        photodiode_fall = Signal()
        self.sync += photodiode_d.eq(ph_diode)
        self.comb += photodiode_fall.eq(~ph_diode & photodiode_d)
        self.sync += If(counter == 0,
                poly_pwm.eq(~poly_pwm),
                counter.eq(int(maxperiod))).Else(
                counter.eq(counter - 1)
                )
        self.submodules.check = FSM(reset_state = "INIT")
        self.check.act("INIT",
                 NextValue(poly_en, 1),
                 NextValue(laser, 1),
                 If(ph_diode ==0,
                   NextState("SUCCESS")
                 )
        )
        self.check.act("SUCCESS",
            NextValue(poly_en, 1),
            NextValue(laser, 0),
            NextState("SUCCESS"),
            If(pin14 ==1,
                   NextState("INIT")
            )
        )

# build
plat = board.Platform()
poly_en = plat.request("poly_en")
poly_pwm = plat.request("poly_pwm")
laser = plat.request("laser0")
ph_diode = plat.request("photodiode")
pin14 = plat.request("gpio14")
build_name = 'photodiode'
photodiode = Photodiode(laser, poly_en, poly_pwm, 10, ph_diode, pin14)
plat.build(photodiode, build_name = build_name)
# upload
plat.upload(build_name)
# remove build dir
plat.removebuild()