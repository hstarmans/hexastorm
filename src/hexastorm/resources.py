import itertools

from amaranth import Record
from amaranth.build import Pins, PinsN, Resource, ResourceError, Subsignal
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


def StepperResource(
    *args, step, direction, limit, number=None, conn=None, attrs=None
):
    """The enable pin is currently activated via the Linux host

    I/O signals:
        O: step           -- pin for step pulse
        O: dir            -- rotating direction of motor
        I: limit          -- limit switch to detect end movement
    """
    io = []
    io.append(
        Subsignal("step", Pins(step, dir="o", conn=conn, assert_width=1))
    )
    io.append(
        Subsignal("dir", Pins(direction, dir="o", conn=conn, assert_width=1))
    )
    io.append(
        Subsignal("limit", Pins(limit, dir="i", conn=conn, assert_width=1))
    )
    if attrs is not None:
        io.append(attrs)
    return Resource.family(*args, number, default_name="stepper", ios=io)


class BLDCLayout(Layout):
    """Layout for BLDC motor"""

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
    """Record for BLDC motor"""

    def __init__(self):
        super().__init__(BLDCLayout())


def BLDCResource(
    *args,
    uL,
    uH,
    vL,
    vH,
    wL,
    wH,
    sensor0,
    sensor1,
    sensor2,
    number=None,
    conn=None,
    attrs=None
):
    """BLDC driver resource

    I/O signals:
        I: *L *H   -- low or high impedance input
                      of BLDC motor
        O: sensor* -- Hall sensors
    """
    io = []
    io.append(Subsignal("uL", Pins(uL, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("uH", Pins(uH, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("vL", Pins(vL, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("vH", Pins(vH, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("wL", Pins(wL, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("wH", Pins(wH, dir="o", conn=conn, assert_width=1)))
    io.append(
        Subsignal("sensor0", Pins(sensor0, dir="i", conn=conn, assert_width=1))
    )
    io.append(
        Subsignal("sensor1", Pins(sensor1, dir="i", conn=conn, assert_width=1))
    )
    io.append(
        Subsignal("sensor2", Pins(sensor2, dir="i", conn=conn, assert_width=1))
    )
    if attrs is not None:
        io.append(attrs)
    return Resource.family(*args, number, default_name="bldc", ios=io)


class LaserScannerLayout(Layout):
    """Layout for laser scanner """
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
    *args, laser0, laser1, photodiode, pwm, enable, number=None, conn=None, attrs=None
):
    """Resource for laser scanner

    The enable pin is currently activated via the Linux host

    I/O signals:
        O: laser0         -- laser channel 0
        O: laser1         -- laser channel 1
        I: photodiode     -- photodiode used to measure position of laser
    """
    io = []
    io.append(
        Subsignal("laser0", Pins(laser0, dir="o", conn=conn, assert_width=1))
    )
    io.append(
        Subsignal("laser1", Pins(laser1, dir="o", conn=conn, assert_width=1))
    )
    io.append(Subsignal("photodiode",
              Pins(photodiode, dir="i", conn=conn, assert_width=1)))
    io.append(Subsignal("pwm", Pins(pwm, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("en", PinsN(enable, dir="o",
                            conn=conn, assert_width=1)))
    if attrs is not None:
        io.append(attrs)
    return Resource.family(*args, number, default_name="laserscanner", ios=io)
