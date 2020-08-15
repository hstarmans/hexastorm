from migen.build.generic_platform import Subsignal, Pins, IOStandard
from migen.build.lattice import LatticePlatform
from migen.build.lattice.programmer import MyStormProgrammer

_io = [
    ("clk100", 0, Pins("61"), IOStandard("LVCMOS33")),
    ("led3", 0, Pins("8"), IOStandard("LVCMOS33")),
    ("led2", 1, Pins("3"), IOStandard("LVCMOS33")),
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
        return MyStormProgrammer(serial_port)
