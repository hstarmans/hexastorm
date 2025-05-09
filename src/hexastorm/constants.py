""" Constants

Settings of the implementation are saved in board.by and constants.py.
Constants are more related to the actual implementation on the FPGA.
"""

from collections import namedtuple, OrderedDict
from math import ceil

COMMANDS = namedtuple(
    "COMMANDS",
    ["EMPTY", "WRITE", "READ", "DEBUG", "POSITION", "START", "STOP"],
)(*range(7))
INSTRUCTIONS = namedtuple(
    "INSTRUCTIONS",
    ["MOVE", "WRITEPIN", "SCANLINE", "LASTSCANLINE"],
)(*range(1, 5))
STATE = namedtuple("STATE", ["FULL", "PARSING", "ERROR"])(*range(3))

COMMAND_BYTES = 1
WORD_BYTES = 8
MEMWIDTH = WORD_BYTES * 8
MOTORFREQ = 1e6  # motor move interpolation freq in Hz
MOVE_TICKS = 10_000  # maximum ticks in move segment

MOVE_INSTRUCTION = {"INSTRUCTION": 1, "TICKS": 7}


class platform:
    memdepth = 256
    name = "firestarter"
    device = "iCE40UP5K"
    package = "SG48"
    default_clk = "SB_HFOSC"
    # This division setting selects the internal oscillator speed:
    # 0: 48MHz, 1: 24MHz, 2: 12MHz, 3: 6MHz.
    clks = {0: 48, 1: 24, 2: 12, 3: 6}
    hfosc_div = 2
    laser_var = {
        "RPM": 3000,
        "SPINUP_TIME": 1.5,
        "STABLE_TIME": 1.125,
        "FACETS": 4,
        "CRYSTAL_HZ": clks[hfosc_div] * 1e6,
        "MOTORDEBUG": "ticksinfacet",
        "MOTORDIVIDER": pow(2, 8),
        "LASER_HZ": 400e3,
        "END%": 0.7,
        "START%": 0.35,
        "SINGLE_LINE": False,
        "DIRECTION": 0,
    }

    def __init__(self, micropython=False):
        self.laser_bits = 1  # enables adding pwm to laser (not widely tested)
        self.poldegree = (
            2  # degree of polynomal to execute move, see movement.py
        )
        # micropython changes order very strangely
        self.stepspermm = OrderedDict()
        self.stepspermm["x"] = 76.2
        self.stepspermm["y"] = 76.2
        self.stepspermm["z"] = 1600
        self.laser_axis = "y"
        self.motors = len(list(self.stepspermm.keys()))
        self.ic_address = 0x28  # spi address
        if micropython:
            self.tmc2209 = {'x': 0, 'y': 1, 'z': 2} # uart ids tmc2209 drivers
            self.scl = 5  # scl pin digipot
            self.sda = 46  # sda pin digipot TODO: should be 4, hotfix to 46
            self.sck = 12
            self.miso = 11
            self.mosi = 13
            self.flash_cs = 10
            self.led_blue = 18
            self.led_red = 8

            # FPGA allows narrow range of baudrates
            # hardware spi between 2 and 3 MHz
            # software spi can be 1 MHz
             
            self.baudrate = int(3e6) # higher, i.e. 3.1 doesn't work
            self.phase = 1        # spi phase must be 1
            self.fpga_cs = 9
            self.enable_pin = 38  # enable pin stepper motors
            self.reset_pin = 47   # can be used to reset FPGA
        # raspberry pi
        else:
            self.ic_dev_nr = 1  # digit pot connection
            self.enable_pin = 17  # enable pin for stepper motors
            self.reset_pin = 26  # can be used to reset FPGA
            self.chip_select = 8  # spi chip select
            self.spi_dev = (0, 0)  # device and channel


def params(platform):
    """determines parameters for laser scanner

    returns dictionary
    """
    var = platform.laser_var
    var["POLY_HZ"] = var["RPM"] / 60
    if platform.name == "Test":
        var["CRYSTAL_HZ"] = round(
            var["TICKSINFACET"] * var["FACETS"] * var["POLY_HZ"]
        )
        var["LASER_HZ"] = var["CRYSTAL_HZ"] / var["LASERTICKS"]
        var["SPINUP_TIME"] = 10 / var["CRYSTAL_HZ"]
        # TODO: stop scanline seems to affect the stable thresh?!
        # can be 30 without stopline (this is from old repo)
        var["STABLE_TIME"] = 5 * var["TICKSINFACET"] / var["CRYSTAL_HZ"]
        var["START%"] = 2 / var["TICKSINFACET"]
        var["END%"] = (var["LASERTICKS"] * var["BITSINSCANLINE"]) / var[
            "TICKSINFACET"
        ] + var["START%"]
        assert var["TICKSINFACET"] == round(
            var["CRYSTAL_HZ"] / (var["POLY_HZ"] * var["FACETS"])
        )
        bitsinscanline = var["BITSINSCANLINE"]
    else:
        var["TICKSINFACET"] = round(
            var["CRYSTAL_HZ"] / (var["POLY_HZ"] * var["FACETS"])
        )
    # parameter creation
    var["LASERTICKS"] = int(var["CRYSTAL_HZ"] / var["LASER_HZ"])
    # jitter requires 2
    # you also need to enable read pin at count one when you read bits
    assert var["LASERTICKS"] > 2
    var['JITTERTICKS'] = round(0.5 * var['LASERTICKS'])
    # PCBmotor
    # var["JITTERTICKS"] = round(0.08 * var["TICKSINFACET"])
    if var["END%"] > round(1 - (var["JITTERTICKS"] + 1) / var["TICKSINFACET"]):
        raise Exception("Invalid settings, END% too high")
    var["BITSINSCANLINE"] = round(
        (var["TICKSINFACET"] * (var["END%"] - var["START%"]))
        / var["LASERTICKS"]
    )
    if platform.name == "Test":
        assert var["BITSINSCANLINE"] == bitsinscanline
    # bits are packed and therefore need to be multiple of 8
    elif (var["BITSINSCANLINE"] % 8) != 0:
        var["BITSINSCANLINE"] += 8 - var["BITSINSCANLINE"] % 8

    if var["BITSINSCANLINE"] <= 0:
        raise Exception("Bits in scanline invalid")
    var["SPINUPTICKS"] = round(var["SPINUP_TIME"] * var["CRYSTAL_HZ"])
    var["STABLETICKS"] = round(var["STABLE_TIME"] * var["CRYSTAL_HZ"])
    # 6 as the original mirror polygon has six sides
    var["POLYPERIOD"] = int(var["CRYSTAL_HZ"] / (var["POLY_HZ"] * 6 * 2))
    return var


def bit_shift(platform):
    """retrieve bit shif for a give degree

    Determined by running the test of
    movement.py and varying parameter
    """
    if platform.poldegree == 3:
        bit_shift = 40
    elif platform.poldegree == 2:
        bit_shift = 25
    else:
        raise Exception("Order not supported")
    return bit_shift


def wordsinscanline(bits):
    """calcuates number of words for a single scanline instruction
    """
    bytesinline = 8  # Instruction, direction, ticksperstep
    bytesinline += ceil(bits / 8)
    return ceil(bytesinline / WORD_BYTES)


def wordsinmove(platform):
    """calcuates number of words for a single move instruction"""
    bytesingcode = (
        sum(MOVE_INSTRUCTION.values())
        + platform.motors * platform.poldegree * WORD_BYTES
    )
    bytesingcode += bytesingcode % WORD_BYTES
    return ceil(bytesingcode / WORD_BYTES)


def getmovedct(platform):
    dct = MOVE_INSTRUCTION
    for i in range(platform.motors):
        for j in range(platform.poldegree):
            dct.update({f"C{i}{j}": 8})
