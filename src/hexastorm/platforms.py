import logging
import os
import platform as pltf
import shlex
import subprocess
from typing import ClassVar

from amaranth.build import Attrs, Clock, Pins, PinsN, Resource, Subsignal
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

logger = logging.getLogger(__name__)


class Firestarter(LatticeICE40Platform):
    """Kicad board: https://github.com/hstarmans/firestarter/"""

    ice40_cfg = PlatformConfig(test=False).ice40_cfg
    device = ice40_cfg["device"]
    package = ice40_cfg["package"]
    default_clk = ice40_cfg["default_clk"]  # or "clk" for the esp32s3 external clock
    hfosc_div = ice40_cfg["hfosc_div"]
    resources: ClassVar[list] = [
        *LEDResources(  # blue, green, red
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
        Resource("fan", 0, Pins("38", dir="o"), Attrs(IO_STANDARD="SB_LVCMOS")),
        Resource(
            "spindle",
            0,
            Pins("42", dir="o"),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
    ]
    connectors: ClassVar[list] = []

    def __init__(self, remote_host=None, device=None):
        LatticeICE40Platform.__init__(self)
        self.esp32_cfg = PlatformConfig(test=False).esp32_cfg
        if device is not None:
            self.esp32_cfg["device"] = device
        self.remote_host = remote_host or os.environ.get("HEXASTORM_REMOTE_HOST")

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
            if self.remote_host:
                logger.info(
                    f"Flashing remotely to {self.remote_host} device {device}..."
                )
                subprocess.check_call(
                    ["scp", bitstream_filename, f"{self.remote_host}:/tmp/fpga.bit"]
                )
                path_env = 'export PATH="$HOME/.local/bin:$HOME/.local/share/uv/tools/mpremote/bin:$PATH"; '
                mpremote_cp = (
                    path_env
                    + f"mpremote resume connect {shlex.quote(device)} fs cp /tmp/fpga.bit :sd/fpga/fpga.bit"
                )
                subprocess.check_call(["ssh", self.remote_host, mpremote_cp])
                for cmd in [
                    "from hexastorm.fpga_host.micropython import ESP32HostSync",
                    "hst = ESP32HostSync()",
                    'hst.flash_fpga("sd/fpga/fpga.bit")',
                ]:
                    mpremote_exec = (
                        path_env
                        + f"mpremote resume connect {shlex.quote(device)} exec {shlex.quote(cmd)}"
                    )
                    subprocess.check_call(["ssh", self.remote_host, mpremote_exec])
                subprocess.run(
                    ["ssh", self.remote_host, "rm -f /tmp/fpga.bit"], check=False
                )
            else:
                logger.info(f"Flashing locally to device {device}...")
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
