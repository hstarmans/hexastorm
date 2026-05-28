import os
import platform as pltf
import subprocess

from amaranth.build import Attrs, Pins, PinsN, Resource, Subsignal, Clock
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

    ice40_cfg = PlatformConfig(test=False).ice40_cfg
    device = ice40_cfg["device"]
    package = ice40_cfg["package"]
    default_clk = ice40_cfg["default_clk"]  # or "clk" for the esp32s3 external clock
    hfosc_div = ice40_cfg["hfosc_div"]
    resources = [
        *LEDResources(
            pins="39 40 41", invert=True, attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),
        Resource(
            "clk",
            0,
            Pins("32", dir="i"),
            Clock(ice40_cfg["clks"][ice40_cfg["hfosc_div"]] * 1e6),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        Resource(
            "fpga_spi",
            0,
            Subsignal("sck", Pins("15")),
            Subsignal("sdi", Pins("17")),
            Subsignal("sdo", Pins("14")),
            Subsignal("cs", PinsN("16")),
            Subsignal("fifo_full", Pins("25")),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # Laserscanner resource
        LaserscannerResource(
            number=0,
            lasers="4 3",  # bit 1, bit 0 !
            pwm="11",
            enable="13",
        ),
        Resource(
            "photodiode",
            0,
            Subsignal("photodiode", Pins("21", dir="i")),
            Attrs(IO_STANDARD="SB_LVCMOS"),
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
            dir_pin="23",
            limit_pin="2",
        ),
        # y-stepper
        StepperResource(
            number=1,
            step_pin="47",
            dir_pin="46",
            limit_pin="48",
        ),
        # z-stepper
        StepperResource(
            number=2,
            step_pin="44",
            dir_pin="43",
            limit_pin="45",
        ),
    ]
    connectors = []

    def __init__(self):
        LatticeICE40Platform.__init__(self)
        self.esp32_cfg = PlatformConfig(test=False).esp32_cfg

    def build(self, *args, **kwargs):
        search_command = "where" if pltf.system() == "Windows" else "which"
        base = f"{search_command} yowasp-"
        os.environ["YOSYS"] = subprocess.getoutput(base + "yosys")
        os.environ["NEXTPNR_ICE40"] = subprocess.getoutput(base + "nextpnr-ice40")
        os.environ["ICEPACK"] = subprocess.getoutput(base + "icepack")
        super().build(*args, **kwargs)

    def toolchain_program(self, products, name, **kwargs):
        device = self.esp32_cfg["device"]
        with products.extract(f"{name}.bin") as bitstream_filename:
            subprocess.check_call(
                [
                    "mpremote",
                    "resume",
                    "connect",
                    device,
                    "fs",
                    "cp",
                    bitstream_filename,
                    ":sd/fpga/fpga.bit",
                ]
            )
            for cmd in [
                "from hexastorm.fpga_host.micropython import ESP32HostSync",
                "hst = ESP32HostSync()",
                'hst.flash_fpga("sd/fpga/fpga.bit")',
            ]:
                subprocess.check_call(
                    ["mpremote", "resume", "connect", device, "exec", cmd]
                )


if __name__ == "__main__":
    Firestarter().build(
        Blinky(),
        do_program=True,
        verbose=True,
    )
