"""
    constraint file for the Kicad board available at
    https://github.com/hstarmans/firestarter/tree/master/pi_hat
"""
import subprocess
import platform
import shutil

from migen.build.generic_platform import Subsignal, Pins, IOStandard
from migen.build.lattice import LatticePlatform

_io = [
    # numbers are for multiple channels, not really used,
    # TODO: wrap polygon in one and laser in one
    ("gpio14", 0, Pins("113"), IOStandard("LVCMOS33")),
    ("gpio15", 0, Pins("112"), IOStandard("LVCMOS33")),
    ("clk100", 0, Pins("61"), IOStandard("LVCMOS33")),
    ("led2", 0, Pins("8"), IOStandard("LVCMOS33")),
    ("led3", 0, Pins("3"), IOStandard("LVCMOS33")),
    ("laser0", 0, Pins("134"), IOStandard("LVCMOS33")),
    ("laser1", 0, Pins("135"), IOStandard("LVCMOS33")),
    ("photodiode", 0, Pins("137"), IOStandard("LVCMOS33")),
    ("poly_en", 0, Pins("141"), IOStandard("LVCMOS33")),
    ("poly_ready", 0, Pins("138"), IOStandard("LVCMOS33")),  # not used
    ("poly_pwm", 0, Pins("139"), IOStandard("LVCMOS33")),
    ("spi", 0,
        Subsignal('cs_n', Pins("85")),
        Subsignal('miso', Pins("87")),
        Subsignal('mosi', Pins("90")),
        Subsignal('clk', Pins("79")),
        IOStandard("LVCMOS33"))
]
_connectors = []


class Platform(LatticePlatform):
    default_clk_name = "clk100"
    default_clk_period = 10
    default_clk_freq = 100  # MHz

    def __init__(self):
        LatticePlatform.__init__(self, "ice40-hx8k-tq144:4k",
                                 _io, _connectors, toolchain="icestorm")

    def build(self, core, build_name, freq=default_clk_freq):
        if 'arm' in platform.machine():  # --pre-pack can't be used due to apio
            print(f"Using apio extension with freq {freq} MHz")
            self.toolchain.nextpnr_build_template = [
                'apio raw "yosys -q -l {build_name}.rpt {build_name}.ys"',
                'apio raw "nextpnr-ice40 {pnr_pkg_opts} \
                    --pcf {build_name}.pcf --json {build_name}.json \
                    --asc {build_name}.txt --freq '+str(freq)+'"',
                'apio raw "icepack {build_name}.txt {build_name}.bin"'
            ]
        return super().build(core, build_name=build_name)

    def upload(self, build_name):
        proc = subprocess.Popen(['icezprog', f'build/{build_name}.bin'],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        stdout, stderr = proc.communicate()
        if stderr or ('Failed' in str(stdout)) or ('Error' in str(stdout)):
            raise Exception("Not able to upload bitstream")

    def removebuild(self):
        shutil.rmtree('build')
