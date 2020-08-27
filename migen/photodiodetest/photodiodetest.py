"""
    photodiode test
    Test for checking function of diode 

    if spinning doesn't stop --> test failed
    
    Rik starmans
"""
from migen.fhdl import *
from migen import *
import sys
sys.path.append("..") 
import hexa as board

class Photodiode(Module):
    def __init__(self, laser, poly_en, poly_pwm, poly_freq, ph_diode):
        '''poly freq is in Hz

        '''
        maxperiod = int(100e6/poly_freq)
        counter = Signal(max=maxperiod)
        #self.sync += poly_en.eq(0)
        self.sync += If(counter == 0,
                poly_pwm.eq(~poly_pwm),
                counter.eq(int(maxperiod))).Else(
                counter.eq(counter - 1)
                )
        self.submodules.check = FSM(reset_state = "INIT")
        self.check.act("INIT",
                 NextValue(poly_en, 0),
                 NextValue(laser, 1),
                 If(ph_diode==0,
                   NextState("SUCCESS")
                 )
        )
        self.check.act("SUCCESS",
            NextValue(poly_en, 1),
            NextValue(laser, 0),
            NextState("SUCCESS")
        )

# build
plat = board.Platform()
poly_en = plat.request("poly_en")
poly_pwm = plat.request("poly_pwm")
laser = plat.request("laser1")
ph_diode = plat.request("ph_diode")
photodiode = Photodiode(laser, poly_en, poly_pwm, 10, ph_diode)
plat.build(photodiode, build_name = 'photodiode')

# # flash
# prog = board.IceStormProgrammer()
# prog.flash(0, "build/top.bin")