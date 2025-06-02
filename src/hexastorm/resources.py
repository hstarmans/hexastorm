import itertools

from amaranth import Record
from amaranth.build import Pins, PinsN, Resource, ResourceError, Subsignal, Attrs
from amaranth.hdl.ast import unsigned
from amaranth.hdl.rec import Layout

__all__ = ["StepperResource", "LaserscannerResource", "BLDCResource"]


def get_all_resources(platform, name):
    """Helper function to get multiple resources by name

    If you request stepper you would end up with a list of
    steppers if there are multiple of such resources
    in your board.
    """
    resources = []
    for number in itertools.count():
        try:
            resources.append(platform.request(name, number))
        except ResourceError:
            break
    return resources


class StepperLayout(Layout):
    """Layout to test stepper motor"""

    def __init__(self):
        super().__init__(
            [("step", unsigned(8)), ("dir", unsigned(16)), ("limit", 1)]
        )


class StepperRecord(Record):
    """Record to test stepper motor"""

    def __init__(self):
        super().__init__(StepperLayout())


def StepperResource(*args, step_pin, dir_pin, limit_pin, number=None, conn=None):
    """
    Stepper motor resource.

    I/O signals:
        step_pin     -- Output step pulse
        dir_pin      -- Output direction control
        limit_pin    -- Input limit switch
    """
    ios = [
        Subsignal("step", Pins(step_pin, dir="o", conn=conn, assert_width=1)),
        Subsignal("dir", Pins(dir_pin, dir="o", conn=conn, assert_width=1)),
        Subsignal("limit", Pins(limit_pin, dir="i", conn=conn, assert_width=1)),
    ]
    ios.append(Attrs(IO_STANDARD="SB_LVCMOS"))

    return Resource.family(*args, number, default_name="stepper", ios=ios)

class BLDCLayout(Layout):
    """Signal layout for a 3-phase BLDC motor."""

    def __init__(self):
        super().__init__(
            [
                ("uL", 1),
                ("uH", 1),
                ("vL", 1),
                ("vH", 1),
                ("wL", 1),
                ("wH", 1),
                ("sensor0", 1),
                ("sensor1", 1),
                ("sensor2", 1),
            ]
        )


class BLDCRecord(Record):
    """Record representation of a BLDC motor resource."""

    def __init__(self):
        super().__init__(BLDCLayout())


def BLDCResource(
    *args,
    uL, uH, vL, vH, wL, wH,
    sensor0, sensor1, sensor2,
    number=None, conn=None,
):
    """
    BLDC motor resource.

    I/O signals:
        uL/uH, vL/vH, wL/wH -- 3-phase low/high drive signals
        sensor*             -- Hall sensor inputs
    """
    ios = [
        Subsignal("uL", Pins(uL, dir="o", conn=conn, assert_width=1)),
        Subsignal("uH", Pins(uH, dir="o", conn=conn, assert_width=1)),
        Subsignal("vL", Pins(vL, dir="o", conn=conn, assert_width=1)),
        Subsignal("vH", Pins(vH, dir="o", conn=conn, assert_width=1)),
        Subsignal("wL", Pins(wL, dir="o", conn=conn, assert_width=1)),
        Subsignal("wH", Pins(wH, dir="o", conn=conn, assert_width=1)),
        Subsignal("sensor0", Pins(sensor0, dir="i", conn=conn, assert_width=1)),
        Subsignal("sensor1", Pins(sensor1, dir="i", conn=conn, assert_width=1)),
        Subsignal("sensor2", Pins(sensor2, dir="i", conn=conn, assert_width=1)),
    ]
    ios.append(Attrs(IO_STANDARD="SB_LVCMOS"))

    return Resource.family(*args, number, default_name="bldc", ios=ios)


class LaserScannerLayout(Layout):
    """Signal layout for a laser scanner head."""
    def __init__(self):
        super().__init__([
            ("laser0", 1),
            ("laser1", 1),
            ("photodiode", 1),
            ("pwm", 1),
            ("en", 1),
        ])

class LaserscannerRecord(Record):
    """Record to test stepper motor"""

    def __init__(self):
        super().__init__(LaserScannerLayout())


def LaserscannerResource(
    *args,
    laser0, laser1, photodiode, pwm, enable,
    number=None, conn=None,
):
    """
    Laser scanner resource.

    I/O signals:
        laser0, laser1   -- Output to laser driver
        photodiode       -- Input signal from scanner sensor
        pwm              -- Output PWM control
        enable           -- Output enable pin (active low)
    """
    ios = [
        Subsignal("laser0", Pins(laser0, dir="o", conn=conn, assert_width=1)),
        Subsignal("laser1", Pins(laser1, dir="o", conn=conn, assert_width=1)),
        Subsignal("photodiode", Pins(photodiode, dir="i", conn=conn, assert_width=1)),
        Subsignal("pwm", Pins(pwm, dir="o", conn=conn, assert_width=1)),
        Subsignal("en", PinsN(enable, dir="o", conn=conn, assert_width=1)),
    ]
    ios.append(Attrs(IO_STANDARD="SB_LVCMOS"))

    return Resource.family(*args, number, default_name="laserscanner", ios=ios)
