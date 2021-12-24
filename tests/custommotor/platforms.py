'''NEW PLATFORM with BLDC motor'''
import os
import subprocess

from nmigen.build import (Resource, Attrs, Pins, PinsN, Clock,
                          Subsignal)
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky

from hexastorm.constants import wordsinmove
from hexastorm.resources import (BLDCResource, BLDCRecord)


class TestPlatform:
    name = 'Test'
    stepspermm = {'x': 400, 'y': 400}
    clks = {0: 1}   # dictionary to determine clock divider, e.g. movement.py
    hfosc_div = 0   # selects clock speed on UP5K and clk divider
    poldegree = 2   # degree of polynomal
    laser_bits = 1
    laser_axis = 'y'
    laser_var = {'RPM': 2000, 'FACETS': 4, 'SINGLE_LINE': False, 
                 'CRYSTAL_HZ': 1000,
                 'TICKSINFACET': 20, 'BITSINSCANLINE': 3, 'LASERTICKS': 4,
                 'SINGLE_FACET': False, 'DIRECTION': 0}
    bldc = BLDCRecord()


class Firestarter(LatticeICE40Platform):
    '''Kicad board available at
       https://github.com/hstarmans/firestarter/
    '''
    name = "firestarter"
    ic_dev_nr = 1      # spi connection
    ic_address = 0x28  # spi address
    enable_pin = 17    # enable pin for stepper motors
    reset_pin = 26     # can be used to reset FPGA
    laser_bits = 1     # enables adding pwm to laser (not widely tested)
    poldegree = 2      # degree of polynomal to execute move, see movement.py
    stepspermm = {'x': 76.2,
                  'y': 76.2,
                  'z': 1600}
    laser_axis = 'y'
    motors = len(stepspermm.keys())
    memdepth = 256
    device = 'iCE40UP5K'
    package = 'SG48'
    default_clk = "SB_HFOSC"
    # This division setting selects the internal oscillator speed:
    # 0: 48MHz, 1: 24MHz, 2: 12MHz, 3: 6MHz.
    clks = {0: 48, 1: 24, 2: 12, 3: 6}
    hfosc_div = 2
    laser_var = {'RPM': 2000, 'SPINUP_TIME': 1.5, 'STABLE_TIME': 1.125,
                 'FACETS': 4, 'CRYSTAL_HZ': clks[hfosc_div]*1E6,
                 'LASER_HZ': 100E3, 'END%': 0.7, 'START%': 0.35,
                 'SINGLE_LINE': False, 'SINGLE_FACET': False,
                 'DIRECTION': 0}
    resources = [
               Resource("clk13", 0, Pins("35", dir="i"),
                        Clock(13.56e6),
                        Attrs(GLOBAL=True, IO_STANDARD="SB_LVCMOS")),
               # TODO: replace with RGB led resource
               *LEDResources(pins='39 40 41', invert=True,
                             attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # NOTE: there is a proper resource in nmigen_boards
               #       this is used as it is also done by luna
               Resource("debug_spi", 0,
                        Subsignal("sck", Pins("18", dir="i")),
                        Subsignal("sdi", Pins("21", dir="i")),
                        Subsignal("sdo", Pins("19", dir="o")),
                        Subsignal("cs", PinsN("13", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")),
               # BLDC driver
               BLDCResource(number=0, uL="25", uH="23", vL="27", vH="26", wL="38",
                            wH="31", sensor="9", 
                            attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               ]
    connectors = []

    def build(self, *args, **kwargs):
        apio = True
        if apio:
            base = 'apio raw "which '
            end = '"'
        else:
            base = 'which yowasp-'
            end = ''
        os.environ['YOSYS'] = subprocess.getoutput(base+'yosys'+end)
        os.environ['NEXTPNR_ICE40'] = \
            subprocess.getoutput(base+'nextpnr-ice40'+end)
        os.environ['ICEPACK'] = subprocess.getoutput(base+'icepack'+end)
        super().build(*args, **kwargs)

    def toolchain_program(self, products, name, **kwargs):
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call(['fomu-flash', '-w', bitstream_filename])
        subprocess.check_call(['fomu-flash', '-r'])


if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True, verbose=True)
