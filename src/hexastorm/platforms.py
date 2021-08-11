""" Firstarter Platform definitions.
To use it, you'll need to set your LUNA_PLATFORM variable:
    > export LUNA_PLATFORM="FPGAG.board:Firestarter"
"""
import os
import subprocess

from nmigen.build import (Resource, Attrs, Pins, PinsN, Clock,
                          Subsignal)
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky

from hexastorm.constants import wordsinmove
from hexastorm.resources import (StepperResource, StepperRecord,
                                 LaserscannerResource, LaserscannerRecord)


class TestPlatform:
    name = 'Test'
    stepspermm = {'x': 400, 'y': 400}
    laser_bits = 1
    laser_axis = 'y'
    laser_var = {'RPM': 2000,
                 'FACETS': 4, 'SINGLE_LINE': False, 'TICKSINFACET': 18,
                 'BITSINSCANLINE': 2, 'LASERTICKS': 4,
                 'SINGLE_FACET': False, 'DIRECTION': 0}
    motors = len(stepspermm.keys())
    memdepth = wordsinmove(motors)*2+1
    steppers = [StepperRecord()]*motors
    laserhead = LaserscannerRecord()


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
    stepspermm = {'x': 76.2,
                  'y': 76.2,
                  'z': 1600}
    laser_axis = 'y'
    laser_var = {'RPM': 2000, 'SPINUP_TIME': 1.5, 'STABLE_TIME': 1.125,
                 'FACETS': 4, 'CRYSTAL_HZ': 50E6, 'LASER_HZ': 100E3,
                 'END%': 0.7, 'START%': 0.35, 'SINGLE_LINE': False,
                 'SINGLE_FACET': False, 'DIRECTION': 0}
    motors = len(stepspermm.keys())
    memdepth = 256
    device = 'iCE40UP5K'
    package = 'SG48'
    default_clk = "SB_HFOSC"
    hfosc_div   = 0
    # This division setting selects the internal oscillator speed:
    # 0: 48MHz, 1: 24MHz, 2: 12MHz, 3: 6MHz.
    #hfosc_div   = 2
    # default_clk = "clk13"
    # clock_domain_generator = FirestarterDomainGenerator
    resources = [
               Resource("clk13", 0, Pins("35", dir="i"),
                        Clock(13.56e6), Attrs(GLOBAL=True, IO_STANDARD="SB_LVCMOS")),
               # TODO: replate with RGB led resource
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
               # Laserscanner resource
               LaserscannerResource(number=0, laser0='32', laser1='31',
                                    photodiode='28',
                                    pwm='26', enable='23',
                                    attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # x-stepper
               StepperResource(number=0, step="6", direction="4",
                               limit="3",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # y-stepper
               StepperResource(number=1, step="2", direction="48",
                               limit="47",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # z-stepper
               StepperResource(number=2, step="46", direction="45",
                               limit="44",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS"))
               ]
    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        # adapted sudo visudo to run program without asking for password
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call(['sudo', 'fomu-flash', '-w', bitstream_filename])
        subprocess.check_call(['sudo', 'fomu-flash', '-r'])
        # this is needed to fix an issue with fomu-flash
        subprocess.check_call(['sudo', 'rmmod', 'spi_bcm2835'])
        subprocess.check_call(['sudo', 'modprobe', 'spi_bcm2835'])

if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True, verbose=True)
