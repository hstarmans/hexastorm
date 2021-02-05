""" Firstarter Platform definitions.
To use it, you'll need to set your LUNA_PLATFORM variable:
    > export LUNA_PLATFORM="FPGAG.board:Firestarter"
"""

import os
import logging
import subprocess


from nmigen import *
from nmigen.build import *
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform

from nmigen_boards.resources import *

from luna.gateware.platform.core import LUNAPlatform


# class FireStarterDomainGenerator(Elaboratable):
#     """ Creates clock domains for the Firestarter. """

#     def elaborate(self, platform):
#         m = Module()

#         # Create our domains...
#         m.domains.sync   = ClockDomain()
#         m.domains.usb    = ClockDomain()
#         m.domains.usb_io = ClockDomain()
#         m.domains.fast   = ClockDomain()

#         # ... ensure our clock is never instantiated with a Global buffer.
#         platform.lookup(platform.default_clk).attrs['GLOBAL'] = False

#         # ... create our 48 MHz IO and 12 MHz USB clocks...
#         clk48 = Signal()
#         clk12 = Signal()
#         #TODO: this is not correct there does exist a better one
#         m.submodules.pll = Instance("SB_PLL40_2F_PAD",
#             i_PACKAGEPIN    = platform.request(platform.default_clk, dir="i"),
#             i_RESETB        = Const(1),
#             i_BYPASS        = Const(0),

#             o_PLLOUTGLOBALA   = clk48,
#             o_PLLOUTGLOBALB   = clk12,

#             # Create a 48 MHz PLL clock...
#             p_FEEDBACK_PATH = "SIMPLE",
#             p_PLLOUT_SELECT_PORTA = "GENCLK",
#             p_PLLOUT_SELECT_PORTB = "SHIFTREG_0deg",
#             p_DIVR          = 0,
#             p_DIVF          = 63,
#             p_DIVQ          = 4,
#             p_FILTER_RANGE  = 1,
#         )

#         # ... and constrain them to their new frequencies.
#         platform.add_clock_constraint(clk48, 48e6)
#         platform.add_clock_constraint(clk12, 12e6)


#         # We'll use our 48MHz clock for everything _except_ the usb domain...
#         m.d.comb += [
#             ClockSignal("usb_io")  .eq(clk48),
#             ClockSignal("fast")    .eq(clk48),
#             ClockSignal("sync")    .eq(clk48),
#             ClockSignal("usb")     .eq(clk12)
#         ]


#         return m


class Firestarter(LatticeICE40Platform, LUNAPlatform):
    '''Kicad board available at
       https://github.com/hstarmans/firestarter/tree/master/pi_hat
    '''
    name = "firestarter"
    device = 'iCE40HX4K'
    package = 'TQ144'
    default_clk = 'clk100_mhz'
    # clock_domain_generator = FireStarterDomainGenerator
    DEFAULT_CLOCK_FREQUENCIES_MHZ = {
        "sync": 100
    }
    resources = [
               Resource("clk100_mhz", 0, Pins("61", dir="i"),
                        Clock(100e6), Attrs(IO_STANDARD="SB_LVCMOS")),
               *LEDResources(pins='8 3',
                             attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # SPI bus connected to the debug controller, for simple register exchanges.
               # Note that the Debug Controller is the controller on this bus.
               Resource("debug_spi", 0,
                        Subsignal("sck",  Pins( "79", dir="i")),
                        Subsignal("sdi",  Pins( "90", dir="i")),
                        Subsignal("sdo",  Pins( "87", dir="o")),
                        Subsignal("cs",   PinsN("85", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               )]

    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        icezprog = os.environ.get("ICEZPROG", "icezprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call([icezprog, bitstream_filename])

if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True)
