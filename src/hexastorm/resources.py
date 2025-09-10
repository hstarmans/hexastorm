import itertools

from amaranth import Record
from amaranth.build import Pins, PinsN, Resource, ResourceError, Subsignal, Attrs
from amaranth.hdl.rec import Layout

__all__ = ["StepperResource", "LaserscannerResource", "BLDCResource"]


def get_all_resources(platform, name, **kwargs):
    """
    Retrieve all available numbered instances of a resource from the platform.

    This function repeatedly requests resources of the given name from the platform,
    incrementing the resource number starting from 0, until a ResourceError is raised.
    It is useful for collecting all resources of a given type (e.g., multiple steppers).

    Parameters:
        platform: The Amaranth platform instance.
        name (str): The base name of the resource to request (e.g., "stepper").
        **kwargs: Optional keyword arguments to pass to `platform.request()`,
                  such as `dir="-"` for RFC 55 compliance.

    Returns:
        List of requested resources, one for each numbered instance found.
    """
    resources = []
    for number in itertools.count():
        try:
            resources.append(platform.request(name, number, **kwargs))
        except ResourceError:
            break
    return resources


class StepperRecord(Record):
    """Record representing a stepper motor interface for simulation or test."""

    def __init__(self):
        layout = [
            ("step", 1),
            ("dir", 1),
            ("limit", 1),
        ]
        super().__init__(layout)


def StepperResource(*args, step_pin, dir_pin, limit_pin, number=None, conn=None):
    """
    Stepper motor resource.

    I/O signals:
        step_pin     -- Step pulse (output)
        dir_pin      -- Direction control (output)
        limit_pin    -- Limit switch (input)
    """
    ios = [
        Subsignal("step", Pins(step_pin, conn=conn, assert_width=1)),
        Subsignal("dir", Pins(dir_pin, conn=conn, assert_width=1)),
        Subsignal("limit", Pins(limit_pin, conn=conn, assert_width=1)),
        Attrs(IO_STANDARD="SB_LVCMOS"),
    ]

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
        super().__init__(
            [
                ("lasers", 2),
                ("pwm", 1),
                ("en", 1),
            ]
        )


class LaserscannerRecord(Record):
    """Record to test stepper motor"""

    def __init__(self):
        super().__init__(LaserScannerLayout())


def LaserscannerResource(
    *args,
    lasers,
    pwm,
    enable,
    number=None,
    conn=None,
):
    """
    Laser scanner resource.

    I/O signals:
        lasers           -- 2-bit output to laser driver (MSB first), e.g. "12 11"
        photodiode       -- Input signal from scanner sensor
        pwm              -- Output PWM control
        enable           -- Output enable pin (active low)
    """
    ios = [
        Subsignal("lasers", Pins(lasers, dir="o", conn=conn, assert_width=2)),
        Subsignal("pwm", Pins(pwm, dir="o", conn=conn, assert_width=1)),
        Subsignal("en", PinsN(enable, dir="o", conn=conn, assert_width=1)),
    ]
    ios.append(Attrs(IO_STANDARD="SB_LVCMOS"))

    return Resource.family(*args, number, default_name="laserscanner", ios=ios)
