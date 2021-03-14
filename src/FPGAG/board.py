""" Firstarter Platform definitions.
To use it, you'll need to set your LUNA_PLATFORM variable:
    > export LUNA_PLATFORM="FPGAG.board:Firestarter"
"""
from math import ceil
import os
import subprocess

from nmigen import Signal
from nmigen.build import Resource, Attrs, Pins, PinsN, Clock, Subsignal
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky

from FPGAG.constants import getbytesinmove
from FPGAG.resources import StepperResource, StepperRecord


class TestPlatform:
    name = 'Test'
    stepspermm = {'x': 400}
    motors = len(stepspermm.keys())
    bytesinmove = getbytesinmove(motors)
    memdepth = ceil(bytesinmove/4)*2
    steppers = [StepperRecord()]
    aux = Signal(8)


class Firestarter(LatticeICE40Platform):
    '''Kicad board available at
       https://github.com/hstarmans/firestarter/tree/master/pi_hat
    '''
    name = "firestarter"
    ic_dev_nr = 1      # spi connection
    ic_address = 0x28  # spi address
    enable_pin = 11    # enable pin for stepper motors
    memdepth = 256
    stepspermm = {'x': 400, 'y': 400, 'z': 400}
    motors = len(stepspermm.keys())
    bytesinmove = getbytesinmove(motors=motors)
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
               # NOTE: there is a proper resource in nmigen_boards
               #       this is used as it is also done by nmigen
               Resource("debug_spi", 0,
                        Subsignal("sck", Pins("79", dir="i")),
                        Subsignal("sdi", Pins("90", dir="i")),
                        Subsignal("sdo", Pins("87", dir="o")),
                        Subsignal("cs", PinsN("85", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")),
               # x-stepper
               StepperResource(number=0, step="38", direction="37",
                               limit='110',
                               attrs=Attrs(IO_STANDARD="SB_LVTTL")),
               # y-stepper
               StepperResource(number=1, step="19", direction="18",
                               limit='124',
                               attrs=Attrs(IO_STANDARD="SB_LVTTL")),
               # z-stepper
               StepperResource(number=2, step="143", direction="142",
                               limit='130',
                               attrs=Attrs(IO_STANDARD="SB_LVTTL"))
               ]
    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        icezprog = os.environ.get("ICEZPROG", "icezprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call([icezprog, bitstream_filename])


if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True, verbose=True)
