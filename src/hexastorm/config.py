"""Platform and SPI Configuration

This module defines constants and utilities for the SPI communication protocol,
as well as platform-level configuration settings used for FPGA synthesis and firmware setup.

- `Spi`: SPI protocol structure (commands, instructions, state bits, and word size logic)
- `PlatformConfig`: High-level platform configuration object for both test and production modes
"""

from collections import OrderedDict
from math import ceil


class Spi:
    """Holds constants and types for the SPI communication protocol."""

    command_bytes = 1
    word_bytes = 8
    move_instruction = dict(instruction=1, ticks=7)

    class Commands:
        """SPI protocol command. Each command is followed by a word."""

        empty = 0
        write = 1
        read = 2
        debug = 3
        position = 4
        start = 5
        stop = 6

    class Instructions:
        """Instruction types encoded in SPI words. Each word can contain a subcommand."""

        move = 1
        write_pin = 2
        scanline = 3
        last_scanline = 4

    class State:
        """State word returned by SPI. Each bit represent a specific status flag."""

        full = 0
        parsing = 1
        error = 2

    def words_scanline(laser_timing):
        """Returns the number of words required for a single scanline instruction."""
        return ceil((8 + ceil(laser_timing["scanline_length"] / 8)) / Spi.word_bytes)

    def words_move(hdl_cfg):
        """Returns the number of words required for a single move instruction."""
        bytes_move = (
            sum(Spi.move_instruction.values())
            + hdl_cfg["motors"] * hdl_cfg["pol_degree"] * Spi.word_bytes
        )
        bytes_move += bytes_move % Spi.word_bytes
        return ceil(bytes_move / Spi.word_bytes)


class PlatformConfig:
    """
    Holds platform configuration.
    """

    def __init__(self, test=False):
        """
        Initialization follows one of two routes:

            - **Test Mode** (`test=True`):
            - You define low-level timing values directly (`ticks_in_facet`, `laser_ticks`, `bits_in_scanline`)
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
        self._hdl_cfg = None
        if test:
            self.laser_timing = dict(
                rpm=1000,
                scanline_length=3,
                facet_ticks=20,
                laser_ticks=4,
            )
        else:
            self.laser_timing = dict(
                rpm=3000,
                spinup_time=1.5,
                stable_time=1.125,
                laser_hz=400e3,
                start_frac=0.35,
                end_frac=0.7,
            )
        self.update_laser_timing()
        self.laser_bits = 1  # enables adding pwm to laser (not widely tested)

    @property
    def esp32_cfg(self):
        """Connections to esp32S3."""
        return dict(
            stepper_cs=38,  # enable pin stepper motors
            tmc2209_uart_ids=dict(x=0, y=1, z=2),
            # sda pin digipot TODO: should be 4, hotfix to 46
            i2c=dict(
                scl=5,
                sda=46,
                digipot_addr=0x28,
            ),
            spi=dict(
                sck=12,
                miso=11,
                mosi=13,
                phase=1,
                polarity=0,
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
    def ice40_cfg(self):
        """Required for LatticeICE40Platform."""
        return dict(
            device="iCE40UP5K",
            package="SG48",
            default_clk="SB_HFOSC",
            hfosc_div="test" if self.test else 2,
            # Not required
            clks={0: 48, 1: 24, 2: 12, 3: 6, "test": 1},
        )

    @property
    def hdl_cfg(self):
        """Required for amaranth synthesis."""
        if self._hdl_cfg is not None:
            return self._hdl_cfg
        if self.test:
            cfg = dict(test=True)
        else:
            cfg = dict(
                test=False,
                mem_depth=256,
            )
        cfg.update(
            dict(
                single_line=False,
                motor_freq=1e6,  # motor move interpolation freq in Hz
                move_ticks=10_000,  # maximum ticks in move segment
                direction=0,  # axis parallel to laser, here x
                motors=len(self.motor_cfg["steps_mm"]),
                motor_divider=pow(2, 8),
                pol_degree=2,
                mem_width=Spi.word_bytes * 8,
                words_scanline=Spi.words_scanline(self.laser_timing),
                motor_debug="ticks_in_facet",
            )
        )

        # Choose bit used for stepping
        if cfg["pol_degree"] == 3:
            cfg["bit_shift"] = 40
        elif cfg["pol_degree"] == 2:
            cfg["bit_shift"] = 25
        else:
            raise ValueError("Only polynomial orders 2 and 3 are supported")

        cfg["words_move"] = Spi.words_move(cfg)
        if self.test:
            cfg["mem_depth"] = cfg["words_move"] * 2 + 1

        self._hdl_cfg = type("Hdl_cfg", (), cfg)()
        return self._hdl_cfg

    @property
    def motor_cfg(self):
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

    def update_laser_timing(self):
        """Update laser timing parameters."""
        # Unpack into locals for brevity
        rpm = self.laser_timing["rpm"]
        facets = 4

        poly_hz = rpm / 60

        if self.test:
            facet_ticks = self.laser_timing["facet_ticks"]
            laser_ticks = self.laser_timing["laser_ticks"]
            scanline_length = self.laser_timing["scanline_length"]

            crystal_hz = round(facet_ticks * facets * poly_hz)
            laser_hz = crystal_hz / laser_ticks
            spinup_time = 10 / crystal_hz
            stable_time = 5 * facet_ticks / crystal_hz
            start_frac = 2 / facet_ticks
            end_frac = (laser_ticks * scanline_length) / facet_ticks + start_frac
            assert facet_ticks == round(crystal_hz / (poly_hz * facets))
        else:
            spinup_time = self.laser_timing["spinup_time"]
            stable_time = self.laser_timing["stable_time"]
            start_frac = self.laser_timing["start_frac"]
            end_frac = self.laser_timing["end_frac"]

            crystal_hz = self.ice40_cfg["clks"][self.ice40_cfg["hfosc_div"]] * 1e6
            facet_ticks = round(crystal_hz / (poly_hz * facets))
            laser_hz = self.laser_timing["laser_hz"]
            laser_ticks = int(crystal_hz / laser_hz)

        spinup_ticks = round(spinup_time * crystal_hz)
        stable_ticks = round(stable_time * crystal_hz)

        jitter_ticks = round(0.5 * laser_ticks)
        scanline_length = round(facet_ticks * (end_frac - start_frac) / laser_ticks)
        motor_period = int(crystal_hz / (poly_hz * 6 * 2))

        # Sanity checks
        assert laser_ticks > 2
        if end_frac > round(1 - (jitter_ticks + 1) / facet_ticks):
            raise Exception("Invalid settings, end_frac too high")

        if self.test:
            assert scanline_length == self.laser_timing["scanline_length"]
        elif scanline_length % 8 != 0:
            scanline_length += 8 - scanline_length % 8

        if scanline_length <= 0:
            raise Exception("Bits in scanline invalid")
        # Update dictionary
        self.laser_timing.update(
            {
                "facets": facets,
                "crystal_hz": crystal_hz,
                "laser_hz": laser_hz,
                "start_frac": start_frac,
                "end_frac": end_frac,
                "facet_ticks": facet_ticks,
                "laser_ticks": laser_ticks,
                "spinup_ticks": spinup_ticks,
                "stable_ticks": stable_ticks,
                "jitter_ticks": jitter_ticks,
                "scanline_length": scanline_length,
                "motor_period": motor_period,
            }
        )


def getmovedct(platform):
    dct = Spi.move_instruction
    for i in range(platform.motors):
        for j in range(platform.poldegree):
            dct.update({f"C{i}{j}": 8})
