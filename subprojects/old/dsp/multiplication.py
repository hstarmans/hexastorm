""" DSP test

Code has no real meaning but it should trigger DSP synthesis.
You can check ICESTORM_DSP 1/8 == 12 percent
"""
import os

from amaranth import Elaboratable, Module, Signal, signed

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
            m.d.sync += self.a.eq(self.a + 1)
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
    os.environ["NMIGEN_synth_opts"] = "-dsp"
    platform = Firestarter()
    # needs to be less than 30 Mhz
    # BEGINT MET INFO
    # LLC 113, DSP 1, pass op 24 MHZ
    platform.hfosc_div = 2
    # print(help(platform.build))
    # look at *.tim
    platform.build(Multest(), do_program=True, verbose=False)

# 16 bit
# Info: Device utilisation:
# Info: 	         ICESTORM_LC:   112/ 5280     2%
# Info: 	        ICESTORM_RAM:     0/   30     0%
# Info: 	               SB_IO:     1/   96     1%
# Info: 	               SB_GB:     2/    8    25%
# Info: 	        ICESTORM_PLL:     0/    1     0%
# Info: 	         SB_WARMBOOT:     0/    1     0%
# Info: 	        ICESTORM_DSP:     1/    8    12%
# Info: 	      ICESTORM_HFOSC:     1/    1   100%
# Info: 	      ICESTORM_LFOSC:     0/    1     0%
# Info: 	              SB_I2C:     0/    2     0%
# Info: 	              SB_SPI:     0/    2     0%
# Info: 	              IO_I3C:     0/    2     0%
# Info: 	         SB_LEDDA_IP:     0/    1     0%
# Info: 	         SB_RGBA_DRV:     0/    1     0%
# Info: 	      ICESTORM_SPRAM:     0/    4     0%


# 32 bit
# Info: Device utilisation:
# Info: 	         ICESTORM_LC:   192/ 5280     3%
# Info: 	        ICESTORM_RAM:     0/   30     0%
# Info: 	               SB_IO:     1/   96     1%
# Info: 	               SB_GB:     4/    8    50%
# Info: 	        ICESTORM_PLL:     0/    1     0%
# Info: 	         SB_WARMBOOT:     0/    1     0%
# Info: 	        ICESTORM_DSP:     2/    8    25%
# Info: 	      ICESTORM_HFOSC:     1/    1   100%
# Info: 	      ICESTORM_LFOSC:     0/    1     0%
# Info: 	              SB_I2C:     0/    2     0%
# Info: 	              SB_SPI:     0/    2     0%
# Info: 	              IO_I3C:     0/    2     0%
# Info: 	         SB_LEDDA_IP:     0/    1     0%
# Info: 	         SB_RGBA_DRV:     0/    1     0%
# Info: 	      ICESTORM_SPRAM:     0/    4     0%
