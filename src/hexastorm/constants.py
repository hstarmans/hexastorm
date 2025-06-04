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

MOTORFREQ = 1e6  # motor move interpolation freq in Hz
MOVE_TICKS = 10_000  # maximum ticks in move segment

MOVE_INSTRUCTION = {"INSTRUCTION": 1, "TICKS": 7}


class PlatformConfig:
    """
    Holds platform configuration. 
    """
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
                rpm = 1000,
                ticks_in_facet = 20,
                bits_in_scanline = 3,
                laser_ticks = 4,
            )
        else:
            self.laser_var = dict(
                rpm = 3000,
                spinup_time = 1.5,
                stable_time = 1.125,
                laser_hz = 400e3,
                start_frac = 0.35,
                end_frac = 0.7,
            )
        self.laser_var.update(dict(
            facets = 4,
            single_line = False,
            motor_divider = pow(2,8),
            motor_debug = "ticks_in_facet",
            direction = 0,
        ))
        self.params()
        self.laser_bits = 1  # enables adding pwm to laser (not widely tested)
    
    @property
    def set_esp32(self):
        return dict(
                tmc2209 = {'x': 0, 'y': 1, 'z': 2}, # uart ids tmc2209 drivers
                scl = 5,  # scl pin digipot
                sda = 4,  # sda pin digipot TODO: should be 4, hotfix to 46
                sck = 12,
                miso = 11,
                mosi = 13,
                ic_address = 0x28,     # spi address
                baudrate = int(2.9e6), # higher, i.e. 3.1 doesn't work
                phase = 1,             # spi phase must be 1
                fpga_cs = 9,
                enable_pin = 38 , # enable pin stepper motors
                reset_pin = 47,   # can be used to reset FPGA
                flash_cs = 10,
                led_blue = 18,
                led_red = 8,)

    @property
    def set_ice40(self):
        """required for LatticeICE40Platform"""
        return dict(
            device = "iCE40UP5K",
            package = "SG48",
            default_clk = "SB_HFOSC",
            hfosc_div = 2,    
        )

    @property
    def build_opts(self):
        """required for amaranth synthesis"""
        if self.test:
            pass
        else:
            return dict(
                name = "firestarter",
                memdepth = 256,
                memwidth = WORD_BYTES * 8,
                poldegree = 2,
                motors = len(self.motor_set['stepspermm']),
            )  

    @property
    def words_scanline(self):
        """Returns the number of words required for a single scanline instruction."""
        return ceil((8 + ceil(self.laser_var['bits_in_scanline'] / 8)) / WORD_BYTES)


    @property
    def words_move(self):
        """Returns the number of words required for a single move instruction."""
        bytesingcode = (
            sum(MOVE_INSTRUCTION.values())
            + self.build_opts['motors'] * self.build_opts['poldegree'] * WORD_BYTES
        )
        bytesingcode += bytesingcode % WORD_BYTES
        return ceil(bytesingcode / WORD_BYTES)
    
    @property
    def motor_set(self):
        if self.test:
            steps = OrderedDict([("x", 400), ("y", 400)])
        else:  
            steps = OrderedDict([
                    ("x", 76.2),
                    ("y", 76.2),
                    ("z", 1600),
                ])
        return dict(
            stepspermm = steps,
            laser_axis = "y",
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
            crystal_hz = clks[self.set_ice40["hfosc_div"]] * 1e6
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

        if self.test == "Test":
            assert bits_in_scanline == self.laser_var["bits_in_scanline"]
        elif bits_in_scanline % 8 != 0:
            bits_in_scanline += 8 - bits_in_scanline % 8

        if bits_in_scanline <= 0:
            raise Exception("Bits in scanline invalid")
        # Update dictionary
        self.laser_var.update({
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
        })

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
    dct = MOVE_INSTRUCTION
    for i in range(platform.motors):
        for j in range(platform.poldegree):
            dct.update({f"C{i}{j}": 8})
