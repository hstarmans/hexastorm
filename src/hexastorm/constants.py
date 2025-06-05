"""Constants

Settings of the implementation are saved in board.by and constants.py.
Constants are more related to the actual implementation on the FPGA.
"""

from collections import OrderedDict
from math import ceil


class SPI:
    """Holds constants and types for the SPI communication protocol."""

    COMMAND_BYTES = 1
    WORD_BYTES = 8
    MOVE_INSTRUCTION = dict(instruction=1, ticks=7)

    class COMMANDS:
        """SPI protocol command. Each command is followed by a word."""

        EMPTY = 0
        WRITE = 1
        READ = 2
        DEBUG = 3
        POSITION = 4
        START = 5
        STOP = 6

    class INSTRUCTIONS:
        """Instruction types encoded in SPI words. Each word can contain a subcommand."""

        MOVE = 1
        WRITEPIN = 2
        SCANLINE = 3
        LASTSCANLINE = 4

    class STATE:
        """State word returned by SPI. Each bit represent a specific status flag."""

        FULL = 0
        PARSING = 1
        ERROR = 2


class PlatformConfig:
    """
    Holds platform configuration.
    """

    MOTORFREQ = 1e6  # motor move interpolation freq in Hz
    MOVE_TICKS = 10_000  # maximum ticks in move segment

    def __init__(self, test=False):
        """
        Initialization follows one of two routes:

            - **Test Mode** (`test=True`):
            - You define low-level timing values directly (`TICKSINFACET`, `LASERTICKS`, `BITSINSCANLINE`)
            - Useful for simulation or FPGA-level tuning

            - **Production Mode** (`test=False`):
            - You define real-world timing values (`LASER_HZ`, `SPINUP_TIME`, etc.)
            - The system computes derived timing parameters

            After initial input, shared constants are injected, and the `params()` method is called to:
            - Normalize both modes into a consistent parameter space
            - Compute all secondary parameters (e.g., `JITTERTICKS`, `POLYPERIOD`)
            - Perform sanity checks (e.g., ensure scanline fits within available ticks)

            The final result is stored in `self.laser_var`, a flat dictionary of all configuration values.
        """
        self.test = test
        if test:
            self.laser_var = dict(
                rpm=1000,
                ticks_in_facet=20,
                bits_in_scanline=3,
                laser_ticks=4,
            )
        else:
            self.laser_var = dict(
                rpm=3000,
                spinup_time=1.5,
                stable_time=1.125,
                laser_hz=400e3,
                start_frac=0.35,
                end_frac=0.7,
            )
        self.laser_var.update(
            dict(
                facets=4,
                single_line=False,
                motor_divider=pow(2, 8),
                motor_debug="ticks_in_facet",
                direction=0,
            )
        )
        self.params()
        self.laser_bits = 1  # enables adding pwm to laser (not widely tested)

    @property
    def esp32_cnfg(self):
        """Connections to esp32S3."""
        return dict(
            stepper_cs=38,  # enable pin stepper motors
            tmc2209_uart_ids=dict(x=0, y=1, z=2),
            # sda pin digipot TODO: should be 4, hotfix to 46
            i2c=dict(
                scl=5,
                sda=4,
                laserdriver_address=0x28,
            ),
            spi=dict(
                sck=12,
                miso=11,
                mosi=13,
                phase=1,
                # higher, i.e. 3 doesn't work
                baudrate=int(2.9e6),
            ),
            fpga_cs=9,
            fpga_reset=47,
            flash_cs=10,
            leds=dict(
                blue=18,
                red=8,
            ),
        )

    @property
    def ice40_cnfg(self):
        """Required for LatticeICE40Platform."""
        return dict(
            device="iCE40UP5K",
            package="SG48",
            default_clk="SB_HFOSC",
            hfosc_div=2,
        )

    @property
    def amaranth_cnfg(self):
        """Required for amaranth synthesis."""
        if self.test:
            pass
        else:
            return dict(
                name="firestarter",
                memdepth=256,
                memwidth=SPI.WORD_BYTES * 8,
                poldegree=2,
                motors=len(self.motor_cnfg["stepspermm"]),
            )

    @property
    def words_scanline(self):
        """Returns the number of words required for a single scanline instruction."""
        return ceil((8 + ceil(self.laser_var["bits_in_scanline"] / 8)) / SPI.WORD_BYTES)

    @property
    def words_move(self):
        """Returns the number of words required for a single move instruction."""
        bytesingcode = (
            sum(SPI.MOVE_INSTRUCTION.values())
            + self.amaranth_cnfg["motors"]
            * self.amaranth_cnfg["poldegree"]
            * SPI.WORD_BYTES
        )
        bytesingcode += bytesingcode % SPI.WORD_BYTES
        return ceil(bytesingcode / SPI.WORD_BYTES)

    @property
    def motor_cnfg(self):
        """Returns the steps per mm and axis orthogonal to laserline."""
        if self.test:
            steps = OrderedDict([("x", 400), ("y", 400)])
        else:
            steps = OrderedDict(
                [
                    ("x", 76.2),
                    ("y", 76.2),
                    ("z", 1600),
                ]
            )
        return dict(
            steps_mm=steps,
            orth2lsrline="y",
        )

    def params(self):
        """Calculate laser timing parameters and store in laser_var."""
        # Unpack into locals for brevity
        rpm = self.laser_var["rpm"]
        facets = self.laser_var["facets"]

        poly_hz = rpm / 60

        if self.test:
            ticks_in_facet = self.laser_var["ticks_in_facet"]
            laser_ticks = self.laser_var["laser_ticks"]
            bits_in_scanline = self.laser_var["bits_in_scanline"]

            crystal_hz = round(ticks_in_facet * facets * poly_hz)
            laser_hz = crystal_hz / laser_ticks
            spinup_time = 10 / crystal_hz
            stable_time = 5 * ticks_in_facet / crystal_hz
            start_frac = 2 / ticks_in_facet
            end_frac = (laser_ticks * bits_in_scanline) / ticks_in_facet + start_frac
            assert ticks_in_facet == round(crystal_hz / (poly_hz * facets))
        else:
            spinup_time = self.laser_var["spinup_time"]
            stable_time = self.laser_var["stable_time"]
            start_frac = self.laser_var["start_frac"]
            end_frac = self.laser_var["end_frac"]

            clks = {0: 48, 1: 24, 2: 12, 3: 6}
            crystal_hz = clks[self.ice40_cnfg["hfosc_div"]] * 1e6
            ticks_in_facet = round(crystal_hz / (poly_hz * facets))
            laser_hz = self.laser_var["laser_hz"]
            laser_ticks = int(crystal_hz / laser_hz)

        spinup_ticks = round(spinup_time * crystal_hz)
        stable_ticks = round(stable_time * crystal_hz)
        jitter_ticks = round(0.5 * laser_ticks)

        bits_in_scanline = round(ticks_in_facet * (end_frac - start_frac) / laser_ticks)
        polyperiod = int(crystal_hz / (poly_hz * 6 * 2))

        # Sanity checks
        assert laser_ticks > 2
        if end_frac > round(1 - (jitter_ticks + 1) / ticks_in_facet):
            raise Exception("Invalid settings, end_frac too high")

        if self.test:
            assert bits_in_scanline == self.laser_var["bits_in_scanline"]
        elif bits_in_scanline % 8 != 0:
            bits_in_scanline += 8 - bits_in_scanline % 8

        if bits_in_scanline <= 0:
            raise Exception("Bits in scanline invalid")
        # Update dictionary
        self.laser_var.update(
            {
                "crystal_hz": crystal_hz,
                "laser_hz": laser_hz,
                "ticks_in_facet": ticks_in_facet,
                "laser_ticks": laser_ticks,
                "spinup_time": spinup_time,
                "stable_time": stable_time,
                "start_frac": start_frac,
                "end_frac": end_frac,
                "spinup_ticks": spinup_ticks,
                "stable_ticks": stable_ticks,
                "jitter_ticks": jitter_ticks,
                "bits_in_scanline": bits_in_scanline,
                "polyperiod": polyperiod,
            }
        )


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


def getmovedct(platform):
    dct = SPI.MOVE_INSTRUCTION
    for i in range(platform.motors):
        for j in range(platform.poldegree):
            dct.update({f"C{i}{j}": 8})
