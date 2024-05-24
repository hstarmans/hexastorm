import os
import platform as pltf
import subprocess

from amaranth import Signal
from amaranth.build import Attrs, Clock, Pins, PinsN, Resource, Subsignal
from amaranth.hdl.mem import Array
from amaranth.vendor import SiliconBluePlatform
from amaranth_boards.resources import LEDResources
from amaranth_boards.test.blinky import Blinky

from .constants import platform, wordsinmove
from .resources import (
    BLDCRecord,
    BLDCResource,
    LaserscannerRecord,
    LaserscannerResource,
    StepperRecord,
    StepperResource,
)


class TestPlatform:
    name = "Test"
    stepspermm = {"x": 400, "y": 400}
    clks = {0: 1}  # dictionary to determine clock divider, e.g. movement.py
    hfosc_div = 0  # selects clock speed on UP5K and clk divider
    poldegree = 2  # degree of polynomal
    laser_bits = 1
    laser_axis = "y"
    laser_var = {
        "RPM": 1000,
        "FACETS": 4,
        "SINGLE_LINE": False,
        "MOTORDEBUG": "ticksinfacet",
        "MOTORDIVIDER": pow(2, 8),
        "TICKSINFACET": 20,
        "BITSINSCANLINE": 3,
        "LASERTICKS": 4,
        "SINGLE_FACET": False,
        "DIRECTION": 0,
    }
    motors = len(stepspermm.keys())
    steppers = [StepperRecord()] * motors
    laserhead = LaserscannerRecord()
    bldc = BLDCRecord()
    leds = Array(Signal() for _ in range(3))

    def __init__(self):
        self.memdepth = wordsinmove(self) * 2 + 1


class Firestarter(SiliconBluePlatform, platform):
    """Kicad board available at
    https://github.com/hstarmans/firestarter/
    """

    name = "firestarter"
    memdepth = 256
    device = "iCE40UP5K"
    package = "SG48"
    default_clk = "SB_HFOSC"
    # This division setting selects the internal oscillator speed:
    # 0: 48MHz, 1: 24MHz, 2: 12MHz, 3: 6MHz.
    clks = {0: 48, 1: 24, 2: 12, 3: 6}
    hfosc_div = 2
    laser_var = {
        "RPM": 2000,
        "SPINUP_TIME": 10,
        "STABLE_TIME": 1.125,
        "FACETS": 4,
        "CRYSTAL_HZ": clks[hfosc_div] * 1e6,
        "MOTORDEBUG": "ticksinfacet",
        "MOTORDIVIDER": pow(2, 8),
        "LASER_HZ": 100e3,
        "END%": 0.7,
        "START%": 0.35,
        "SINGLE_LINE": False,
        "SINGLE_FACET": False,
        "DIRECTION": 0,
    }

    # default_clk = "clk13"
    # clock_domain_generator = FirestarterDomainGenerator
    resources = [
        Resource(
            "clk13",
            0,
            Pins("35", dir="i"),
            Clock(13.56e6),
            Attrs(GLOBAL=True, IO_STANDARD="SB_LVCMOS"),
        ),
        # TODO: replace with RGB led resource
        *LEDResources(
            pins="39 40 41", invert=True, attrs=Attrs(IO_STANDARD="SB_LVCMOS")
        ),
        # NOTE: there is a proper resource in nmigen_boards
        #       this is used as it is also done by luna
        Resource(
            "debug_spi",
            0,
            Subsignal("sck", Pins("18", dir="i")),
            Subsignal("sdi", Pins("21", dir="i")),
            Subsignal("sdo", Pins("19", dir="o")),
            Subsignal("cs", PinsN("13", dir="i")),
            Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # Laserscanner resource
        LaserscannerResource(
            number=0,
            laser0="31",
            laser1="28",
            photodiode="38",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # BLDC driver
        BLDCResource(
            number=0,
            uL="25",
            uH="26",
            vL="9",
            vH="23",
            wL="27",
            wH="32",
            sensor0="34",
            sensor1="36",
            sensor2="37",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # x-stepper
        StepperResource(
            number=0,
            step="6",
            direction="4",
            limit="44",  # x and z are switched
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # y-stepper
        StepperResource(
            number=1,
            step="2",
            direction="48",
            limit="47",
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
        # z-stepper
        StepperResource(
            number=2,
            step="46",
            direction="45",
            limit="3",  # x and z are switched
            attrs=Attrs(IO_STANDARD="SB_LVCMOS"),
        ),
    ]
    connectors = []

    def __init__(self, micropython=False):
        SiliconBluePlatform.__init__(self)
        platform.__init__(self, micropython)

    def build(self, *args, **kwargs):
        if pltf.system() == "Windows":
            search_command = "where"
        else:
            search_command = "which"
        base = f"{search_command} yowasp-"
        end = ""
        # yowasp-yosys doesn't work
        # os.environ['YOSYS'] = subprocess.getoutput(base+'yosys'+end)
        os.environ["NEXTPNR_ICE40"] = subprocess.getoutput(
            base + "nextpnr-ice40" + end
        )
        os.environ["ICEPACK"] = subprocess.getoutput(base + "icepack" + end)
        super().build(*args, **kwargs)

    def toolchain_program(self, products, name, **kwargs):
        with products.extract("{}.bin".format(name)) as bitstream_filename:
            subprocess.check_call(["fomu-flash", "-w", bitstream_filename])
        subprocess.check_call(["fomu-flash", "-r"])


if __name__ == "__main__":
    Firestarter().build(Blinky(), do_program=True, verbose=True)
