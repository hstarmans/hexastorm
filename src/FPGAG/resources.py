import itertools

from nmigen import Record

from nmigen.hdl.ast import unsigned
from nmigen.build import Subsignal, Resource, Pins, ResourceError
from nmigen.hdl.rec import Layout


__all__ = [
    "StepperResource", 
    "LaserscannerResource"
]


def get_all_resources(platform, name):
    """ Helper function to get multiple resources by name
    
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
    """ Layout to test stepper motor """
    def __init__(self):
            super().__init__([
                ("step", unsigned(8)),
                ("dir", unsigned(16)),
                ("limit", 1)
            ])

class StepperRecord(Record):
    """ Record to test stepper motor """
    def __init__(self):
        super().__init__(StepperLayout())


def StepperResource(*args, step, direction, limit, number=None,
                    conn=None, attrs=None):
    """ The enable pin is currently activated via the Linux host

    I/O signals:
        O: step           -- pin for step pulse 
        O: dir            -- rotating direction of motor
        I: limit          -- limit switch to detect end movement
    """
    io = []
    io.append(Subsignal("step", Pins(step, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("dir", Pins(direction, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("limit", Pins(limit, dir="i", conn=conn, assert_width=1)))
    if attrs is not None:
        io.append(attrs)
    print(f"creating {number}")
    return Resource.family(*args, number, default_name="stepper", ios=io)


def LaserscannerResource(*args, laser, photodiode, pwm, number=None,
                         conn=None, attrs=None):
    """ Resource for laser scanner
    
    The enable pin is currently activated via the Linux host

    I/O signals:
        O: laser          -- turn on laser 
        I: photodiode     -- photodiode used to measure position of laser
        O: pwm            -- pwm pin to rotate polygon
    """
    io = []
    io.append(Subsignal("laser", Pins(laser, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("photodiode", Pins(photodiode, dir="o", conn=conn, assert_width=1)))
    io.append(Subsignal("pwm", Pins(pwm, dir="o", conn=conn, assert_width=1)))
    if attrs is not None:
        io.append(attrs)
    return Resource.family(*args, default_name="Laserscanner", number=number, ios=io)
