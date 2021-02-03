import os
import subprocess

from nmigen.build import Resource, Pins, Attrs, Clock
from nmigen.vendor.lattice_ice40 import LatticeICE40Platform
from nmigen_boards.resources import LEDResources
from nmigen_boards.test.blinky import Blinky


__all__ = ['Firestarter']


class Firestarter(LatticeICE40Platform):
    '''Kicad board available at
       https://github.com/hstarmans/firestarter/tree/master/pi_hat
    '''
    device = 'iCE40HX4K'
    package = 'TQ144'
    default_clk = 'clk100'
    resources = [
               Resource("clk100", 0, Pins("61", dir="i"),
                        Clock(100e6), Attrs(IO_STANDARD="SB_LVCMOS")),
               *LEDResources(pins='8 3',
                             attrs=Attrs(IO_STANDARD="SB_LVCMOS"))]

    connectors = []

    def toolchain_program(self, products, name, **kwargs):
        icezprog = os.environ.get("ICEZPROG", "icezprog")
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call([icezprog, bitstream_filename])


if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True)
