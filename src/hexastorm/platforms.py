import logging
import os
import shlex
import shutil
import subprocess
from typing import ClassVar

from amaranth.build import Attrs, Clock, Pins, PinsN, Resource, Subsignal
from amaranth.vendor import LatticeICE40Platform

from .config import PlatformConfig
from .resources import LaserscannerResource, StepperResource

logger = logging.getLogger(__name__)


class Firestarter(LatticeICE40Platform):
    """Kicad board: https://github.com/hstarmans/firestarter/"""

    ice40_cfg = PlatformConfig(test=False).ice40_cfg
    device = ice40_cfg["device"]
    package = ice40_cfg["package"]
    default_clk = ice40_cfg["default_clk"]  # or "clk" for the esp32s3 external clock
    hfosc_div = ice40_cfg["hfosc_div"]
    resources: ClassVar[list] = [
        *[
            Resource("led", i, PinsN(pin, dir="o"), Attrs(IO_STANDARD="SB_LVCMOS"))
            for i, pin in enumerate(("39", "40", "41"))
        ],  # blue, green, red
        Resource(
            "clk",
            0,
            Pins("32", dir="i"),
            Clock(ice40_cfg["clks"]["esp32s3"] * 1e6),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        Resource(
            "fpga_spi",
            0,
            Subsignal("sck", Pins("15", dir="i")),
            Subsignal("sdi", Pins("17", dir="i")),
            Subsignal("sdo", Pins("14", dir="o")),
            Subsignal("cs", PinsN("16", dir="i")),
            Subsignal("fifo_full", Pins("25", dir="o")),
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

    def __init__(self, remote_host=None, device=None, **kwargs):
        super().__init__(**kwargs)
        self.esp32_cfg = PlatformConfig(test=False).esp32_cfg
        if device is not None:
            self.esp32_cfg["device"] = device
        self.remote_host = remote_host or os.environ.get("HEXASTORM_REMOTE_HOST")

    def _configure_yowasp_tools(self):
        for env_var, tool_name in (
            ("YOSYS", "yowasp-yosys"),
            ("NEXTPNR_ICE40", "yowasp-nextpnr-ice40"),
            ("ICEPACK", "yowasp-icepack"),
        ):
            if env_var not in os.environ:
                tool_path = shutil.which(tool_name)
                if tool_path:
                    os.environ[env_var] = tool_path

    def build(self, *args, **kwargs):
        self._configure_yowasp_tools()
        return super().build(*args, **kwargs)

    def toolchain_program(
        self, products, name, *, device=None, remote_host=None, **kwargs
    ):
        target_device = device or self.esp32_cfg["device"]
        target_remote = remote_host if remote_host is not None else self.remote_host
        flash_cmd = (
            "from hexastorm.fpga_host.micropython import ESP32HostSync; "
            "ESP32HostSync().flash_fpga('sd/fpga/fpga.bit')"
        )
        with products.extract(f"{name}.bin") as bitstream_filename:
            if target_remote:
                logger.info(
                    f"Flashing remotely to {target_remote} device {target_device}..."
                )
                subprocess.check_call(
                    ["scp", bitstream_filename, f"{target_remote}:/tmp/fpga.bit"]
                )
                path_env = (
                    'export PATH="$HOME/.local/bin:'
                    '$HOME/.local/share/uv/tools/mpremote/bin:$PATH"; '
                )
                q_device = shlex.quote(target_device)
                q_flash = shlex.quote(flash_cmd)
                remote_cmd = (
                    f"{path_env}"
                    f"mpremote resume connect {q_device} "
                    f"fs cp /tmp/fpga.bit :sd/fpga/fpga.bit + exec {q_flash}"
                )
                try:
                    subprocess.check_call(["ssh", target_remote, remote_cmd])
                finally:
                    subprocess.run(
                        ["ssh", target_remote, "rm -f /tmp/fpga.bit"], check=False
                    )
            else:
                logger.info(f"Flashing locally to device {target_device}...")
                subprocess.check_call(
                    [
                        "mpremote",
                        "resume",
                        "connect",
                        target_device,
                        "fs",
                        "cp",
                        bitstream_filename,
                        ":sd/fpga/fpga.bit",
                    ]
                )
                subprocess.check_call(
                    [
                        "mpremote",
                        "resume",
                        "connect",
                        target_device,
                        "exec",
                        flash_cmd,
                    ]
                )


if __name__ == "__main__":
    from amaranth import Elaboratable, Module, Signal
    from amaranth.lib.io import Buffer

    class _Blinky(Elaboratable):
        def elaborate(self, platform):
            m = Module()
            led_buf = Buffer("o", platform.request("led", 0, dir="-"))
            m.submodules.led_buf = led_buf
            counter = Signal(24)
            m.d.sync += counter.eq(counter + 1)
            m.d.comb += led_buf.o.eq(counter[-1])
            return m

    Firestarter().build(
        _Blinky(),
        do_program=True,
        verbose=True,
    )
