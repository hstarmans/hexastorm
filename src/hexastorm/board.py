"""
    constraint file for the Kicad board available at https://github.com/hstarmans/firestarter/tree/master/pi_hat
"""
import platform

from migen.build.generic_platform import Subsignal, Pins, IOStandard
from migen.build.lattice import LatticePlatform
from migen.build.lattice.programmer import IceStormProgrammer

_io = [
    ("clk100", 0, Pins("61"), IOStandard("LVCMOS33")),
    ("led2", 0, Pins("8"), IOStandard("LVCMOS33")),
    ("led3", 1, Pins("3"), IOStandard("LVCMOS33")),
    ("laser0", 1, Pins("134"), IOStandard("LVCMOS33")),
    ("laser1", 1, Pins("135"), IOStandard("LVCMOS33")),
    ("photodiode", 1, Pins("137"), IOStandard("LVCMOS33")),
    ("poly_en", 1, Pins("141"), IOStandard("LVCMOS33")),
    ("poly_ready", 1, Pins("138"), IOStandard("LVCMOS33")),  # not used
    ("poly_pwm", 1, Pins("139"), IOStandard("LVCMOS33")),
    ("spi", 0,
        Subsignal('cs_n',Pins("85")),
        Subsignal('miso',Pins("87")),
        Subsignal('mosi',Pins("90")),
        Subsignal('clk',Pins("79")),
        IOStandard("LVCMOS33"))
]

_connectors = []


class Platform(LatticePlatform):
    default_clk_name = "clk100"
    default_clk_period = 10

    def __init__(self):
        LatticePlatform.__init__(self, "ice40-hx8k-tq144:4k",
                                 _io, _connectors, toolchain="icestorm")        

    def create_programmer(self, serial_port="/dev/ttyACM0"):
        return IceStormProgrammer()