""" DSP test

Code has no real meaning but it should trigger DSP synthesis.
You can check ICESTORM_DSP 1/8 == 12 percent
"""
import os
from nmigen import Elaboratable, Module, Signal, signed
from hexastorm.platforms import Firestarter


class Multest(Elaboratable):
    def __init__(self):
        self.a = Signal(signed(16))
        self.b = Signal(signed(16))
        self.c = Signal(signed(32))

    def elaborate(self, platform):
        m = Module()
        led0 = platform.request("led", 0)

        with m.If(self.c > 10):
            m.d.sync += self.a.eq(self.a+1)
        with m.Else():
            m.d.sync += self.a.eq(1)

        m.d.sync += self.b.eq(2)
        m.d.sync += self.c.eq(self.a * self.b)
        with m.If(self.c > 10):
            m.d.comb += led0.eq(0)
        with m.Else():
            m.d.comb += led0.eq(1)
        return m


if __name__ == "__main__":
    # Force yosys to use DSP slices.
    os.environ['NMIGEN_synth_opts'] = '-dsp'
    platform = Firestarter()
    # needs to be less than 30 Mhz
    platform.hfosc_div = 2
    platform.build(Multest(),
                   do_program=False,
                   verbose=True)
