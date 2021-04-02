""" Firstarter Platform definitions.
To use it, you'll need to set your LUNA_PLATFORM variable:
    > export LUNA_PLATFORM="FPGAG.board:Firestarter"
"""
from math import ceil
import os
import subprocess

from nmigen import Signal, Elaboratable, Module, ClockDomain, Const, Instance
from nmigen.build import (Resource, Attrs, Pins, PinsN, Clock,
                          Subsignal)
from nmigen.hdl import ClockSignal, ResetSignal
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky

from FPGAG.constants import getbytesinmove, WORD_BYTES
from FPGAG.resources import StepperResource, StepperRecord


class TestPlatform:
    name = 'Test'
    stepspermm = {'x': 400} #'y': 400}
    motors = len(stepspermm.keys())
    bytesinmove = getbytesinmove(motors)
    memdepth = ceil(bytesinmove/WORD_BYTES)*2
    steppers = [StepperRecord()]
    aux = Signal(8)


# other option is kbob nmigen-examples nmigen_lib pll
class FirestarterDomainGenerator(Elaboratable):
    """ Creates clock domains for Firestarter

    A description of clock options;
    https://github.com/nmigen/nmigen/issues/425
    """
    nclkouts_max = 1
    divr_range = (0,  16)
    divf_range = (0, 128)
    divq_range = (0,   7)
    clki_freq_range = (10e6, 133e9)
    clko_freq_range = (16e6,  275e9)
    vco_freq_range = (533e6, 1066e6)

    def __init__(self, *, clock_frequencies=None, clock_signal_name=None):
        pass

    def compute_config(self, clkin_freq, clkout_freq, margin=1e-2):
        ''' compute settings for desired clkout given clkin

        clkin_freq  -- clock in frequency in MHz
        clkout_freq -- clock out frequency in MHz
        margin      -- max error, 1e-2 is 1 percent
        '''
        (clki_freq_min, clki_freq_max) = self.clki_freq_range
        assert clkin_freq >= clki_freq_min
        assert clkin_freq <= clki_freq_max
        config = {}
        for divr in range(*self.divr_range):
            for divf in range(*self.divf_range):
                all_valid = True
                vco_freq = clkin_freq/(divr + 1)*(divf + 1)
                (vco_freq_min, vco_freq_max) = self.vco_freq_range
                if vco_freq >= vco_freq_min and vco_freq <= vco_freq_max:
                    f = clkout_freq
                    valid = False
                    for divq in range(*self.divq_range):
                        clk_freq = vco_freq/(2**divq)
                        if abs(clk_freq - f) <= f*margin:
                            config["clkout_freq"] = clk_freq
                            config["divq"] = divq
                            valid = True
                            break
                    if not valid:
                        all_valid = False
                else:
                    all_valid = False
                if all_valid:
                    config["vco"] = vco_freq
                    config["divr"] = divr
                    config["divf"] = divf
                    for f, v in [(17e6, 1), (26e6, 2), (44e6, 3),
                                 (66e6, 4), (101e6, 5), (133e6, 6)]:
                        pfd_freq = clkin_freq/(config["divr"] + 1)
                        if pfd_freq < f:
                            config['filter_range'] = v
                            break
                    return config
        raise ValueError("No PLL config found")

    def elaborate(self, platform):
        m = Module()
        locked = Signal()

        # Create our domains...
        m.domains.sync = ClockDomain()
        m.domains.pol = ClockDomain()

        # clocks 50 MHz circuit
        #         1 MHz update frequency motor
        clk50 = Signal()
        # clk1 = Signal()
        # details see iCE40 sysCLOCK PLL Design and Usage
        # settings comment out are for SB_PLL40_2F_CORE
        m.submodules.pll = \
            Instance("SB_PLL40_CORE",
                     i_REFERENCECLK=platform.request(platform.default_clk),
                     i_RESETB=Const(1),
                     # i_BYPASS=Const(0),
                     o_PLLOUTGLOBAL=clk50,
                     o_LOCK=locked,
                     # Create a 50 MHz PLL clock...
                     p_FEEDBACK_PATH="SIMPLE",
                     # internally generated PLL
                     # p_PLLOUT_SELECT_PORTA="GENCLK",
                     # p_PLLOUT_SELECT_PORTB="SHIFTREG_0deg",
                     p_DIVR=0,
                     p_DIVF=7,
                     p_DIVQ=4,
                     p_FILTER_RANGE=5,
                     )

        # ... and constrain them to their new frequencies.
        platform.add_clock_constraint(clk50, 50e6)
        # platform.add_clock_constraint(clk1, 1e6)

        # We'll use our 50MHz clock for everything _except_ the polynomal
        # which create ticks for the motors
        m.d.comb += [
            #ClockSignal("pol").eq(clk1),
            ClockSignal("sync").eq(clk50),
            #ResetSignal("pol").eq(~locked),
            ResetSignal("sync").eq(~locked),
        ]

        return m


class Firestarter(LatticeICE40Platform):
    '''Kicad board available at
       https://github.com/hstarmans/firestarter/tree/master/pi_hat
    '''
    name = "firestarter"
    ic_dev_nr = 1      # spi connection
    ic_address = 0x28  # spi address
    enable_pin = 17    # enable pin for stepper motors
    reset_pin = 26     # can be used to reset FPGA
    memdepth = 256
    stepspermm = {'x': 400, 'y': 400, 'z': 400}
    motors = len(stepspermm.keys())
    bytesinmove = getbytesinmove(motors=motors)
    device = 'iCE40HX4K'
    package = 'TQ144'
    default_clk = 'clk100_mhz'
    clock_domain_generator = FirestarterDomainGenerator
    resources = [
               Resource("clk100_mhz", 0, Pins("61", dir="i"),
                        Clock(100e6), Attrs(IO_STANDARD="SB_LVCMOS")),
               *LEDResources(pins='8 3',
                             attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # NOTE: there is a proper resource in nmigen_boards
               #       this is used as it is also done by luna
               Resource("debug_spi", 0,
                        Subsignal("sck", Pins("79", dir="i")),
                        Subsignal("sdi", Pins("90", dir="i")),
                        Subsignal("sdo", Pins("87", dir="o")),
                        Subsignal("cs", PinsN("85", dir="i")),
                        Attrs(IO_STANDARD="SB_LVCMOS")),
               # x-stepper
               StepperResource(number=0, step="38", direction="37",
                               limit="110",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # y-stepper
               StepperResource(number=1, step="19", direction="18",
                               limit="124",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
               # z-stepper
               StepperResource(number=2, step="143", direction="142",
                               limit="130",
                               attrs=Attrs(IO_STANDARD="SB_LVCMOS"))
               ]
    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        icezprog = os.environ.get("ICEZPROG", "icezprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call([icezprog, bitstream_filename])


if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True, verbose=True)
