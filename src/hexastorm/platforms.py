import os
import platform as pltf
import subprocess

from amaranth.build import Attrs, Pins, PinsN, Resource, Subsignal
from amaranth.vendor import LatticeICE40Platform
from amaranth_boards.resources import LEDResources
from amaranth_boards.test.blinky import Blinky

from .config import PlatformConfig
from .resources import (
    # BLDCRecord,
    # BLDCResource,
    LaserscannerResource,
    StepperResource,
)


class Firestarter(LatticeICE40Platform):
    """Kicad board: https://github.com/hstarmans/firestarter/"""

    cfg = PlatformConfig(test=False).ice40_cfg
    device = cfg["device"]
    package = cfg["package"]
    default_clk = cfg["default_clk"]
    hfosc_div = cfg["hfosc_div"]

    resources = [
        *LEDResources(pins="39", invert=True, attrs=Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource(
            "debug_spi",
            0,
            Subsignal("sck", Pins("19")),
            Subsignal("sdi", Pins("13")),
            Subsignal("sdo", Pins("18")),
            Subsignal("cs", PinsN("25")),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # Laserscanner resource
        LaserscannerResource(
            number=0,
            lasers="12 11",  # bit 1, bit 0 !
            photodiode="46",
            pwm="6",
            enable="4",
        ),
        # # BLDC driver
        # BLDCResource(
        #     number=0,
        #     uL="2",
        #     uH="4",
        #     vL="3",
        #     vH="6",
        #     wL="9",
        #     wH="10",
        #     sensor0="45",
        #     sensor1="47",
        #     sensor2="48",
        # ),
        # x-stepper
        StepperResource(
            number=0,
            step_pin="26",
            dir_pin="20",
            limit_pin="42",
        ),
        # y-stepper
        StepperResource(
            number=1,
            step_pin="37",
            dir_pin="36",
            limit_pin="21",
        ),
        # z-stepper
        StepperResource(
            number=2,
            step_pin="35",
            dir_pin="27",
            limit_pin="23",
        ),
    ]
    connectors = []

    def __init__(self):
        LatticeICE40Platform.__init__(self)

    def build(self, *args, **kwargs):
        search_command = "where" if pltf.system() == "Windows" else "which"
        base = f"{search_command} yowasp-"
        os.environ["YOSYS"] = subprocess.getoutput(base + "yosys")
        os.environ["NEXTPNR_ICE40"] = subprocess.getoutput(base + "nextpnr-ice40")
        os.environ["ICEPACK"] = subprocess.getoutput(base + "icepack")
        super().build(*args, **kwargs)

    def toolchain_program(self, products, name, **kwargs):
        with products.extract(f"{name}.bin") as bitstream_filename:
            subprocess.check_call(
                [
                    "mpremote",
                    "resume",
                    "connect",
                    self.micropython,
                    "fs",
                    "cp",
                    bitstream_filename,
                    ":sd/fpga/fpga.bit",
                ]
            )
            for cmd in [
                "from hexastorm.controller import Host",
                "hst = Host(micropython=True)",
                'hst.flash_fpga("sd/fpga/fpga.bit")',
            ]:
                subprocess.check_call(
                    ["mpremote", "resume", "connect", self.micropython, "exec", cmd]
                )


if __name__ == "__main__":
    Firestarter(micropython="/dev/ttyACM0").build(
        Blinky(),
        do_program=False,
        verbose=True,
    )
