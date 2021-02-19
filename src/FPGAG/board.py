""" Firstarter Platform definitions.
To use it, you'll need to set your LUNA_PLATFORM variable:
    > export LUNA_PLATFORM="FPGAG.board:Firestarter"
"""

import os
import subprocess

from nmigen.build import Resource, Attrs, Pins, PinsN, Clock, Subsignal
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky
from luna.gateware.platform.core import LUNAPlatform


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
                        Subsignal("sck", Pins( "79", dir="i")),
                        Subsignal("sdi", Pins( "90", dir="i")),
                        Subsignal("sdo", Pins( "87", dir="o")),
                        Subsignal("cs", PinsN("85", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               ),
               #TODO: fix this by creating proper resource!!
               Resource("AUX", 0,
                        Subsignal("0", Pins("39", dir="o")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               ),
               Resource("DIRECTIONS", 0,
                        Subsignal("x", Pins("37", dir="o")),
                        Subsignal("y", Pins("18", dir="o")),
                        Subsignal("z", Pins("142", dir="o")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               ),
               Resource("STEPS", 0,
                        Subsignal("x", Pins("38", dir="o")),
                        Subsignal("y", Pins("19", dir="o")),
                        Subsignal("z", Pins("143", dir="o")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               ),
               Resource("LIMITS", 0,
                        Subsignal("limitx", Pins("110", dir="i")),
                        Subsignal("limity", Pins("124", dir="i")),
                        Subsignal("limitz", Pins("130", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")
               )
               ]

    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        icezprog = os.environ.get("ICEZPROG", "icezprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call([icezprog, bitstream_filename])

if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True)
