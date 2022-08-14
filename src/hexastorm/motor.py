import itertools
import unittest

from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.hdl.mem import Array
from amaranth.build import ResourceError

from .platforms import TestPlatform
from .resources import get_all_resources


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor

       frequency in Hz, rotation frequency of BLDC motor
       state of the motor, wether motor is turned on
       I: enable_prism   -- enable pin scanner motor
       I: Hall (3)       -- hall sensor measure state motor
       O: leds (3)       -- display measurements Hall sensors
       O: bldc (6)       -- powers BLDC motor
    '''

    def __init__(self, platform,
                 top=False):
        """
        platform  -- pass test platform
        top       -- True if top module
        """
        self.platform = platform
        self.top = top
        self.enable_prism = Signal()
        self.leds = Array(Signal() for _ in range(3))
        self.hall = Array(Signal() for _ in range(3))
        # powers bridges motor
        self.uL = Signal()
        self.uH = Signal()
        self.vL = Signal()
        self.vH = Signal()
        self.wL = Signal()
        self.wH = Signal()

    def elaborate(self, platform):
        m = Module()
        state = Signal(3)

        if platform and self.top:
            leds = [res.o for res in get_all_resources(platform, "led")]
            for idx in range(len(self.leds)):
                m.d.comb += self.leds[idx].eq(leds[idx])
            bldc = platform.request("bldc")
            m.d.comb += self.enable_prism.eq(1)
            m.d.comb += [bldc.uL.eq(self.uL),
                         bldc.uH.eq(self.uH),
                         bldc.vL.eq(self.vL),
                         bldc.vH.eq(self.vH),
                         bldc.wL.eq(self.wL),
                         bldc.wH.eq(self.wH)]
            m.d.comb += [self.hall[0].eq(bldc.sensor0),
                         self.hall[1].eq(bldc.sensor1),
                         self.hall[2].eq(bldc.sensor2)]

        for idx in range(len(self.leds)):
            m.d.comb += self.leds[idx].eq(self.hall[idx])

        # tested by trying out all possibilities
        m.d.sync += state.eq(Cat(self.hall[0],
                                 self.hall[1],
                                 self.hall[2]))

        target = 200000
        max_measurement = target*10
        measurement = Signal(range(max_measurement))
        timer = Signal().like(measurement)
        statefilter = Signal(3)
        stateold = Signal(3)
        max_delay = 1000
        delay = Signal(range(max_delay))

        # Statefilter
        # The current motor can reach state 0,0,0
        # The measurements of the hall sensors are
        # filtered to prevent this
        with m.If((state >= 1) & (state <= 6)):
            m.d.sync += statefilter.eq(state)

        m.d.sync += stateold.eq(statefilter)

        # PID controller
        step = max_delay//100
        with m.If((measurement > target) & (delay >= step)):
            m.d.sync += delay.eq(delay - step)
        with m.Elif((measurement < target) & (delay < (max_delay-step))):
            m.d.sync += delay.eq(delay+step)
        with m.Else():
            m.d.sync += delay.eq(0)

        # Measure Cycle
        # Measures the speed of the motor by checking
        # how often state 1 is reached
        with m.If(timer >= max_measurement-1):
            m.d.sync += [timer.eq(0),
                         measurement.eq(timer)]
        with m.Elif((statefilter == 1) & (stateold != 1)):
            m.d.sync += [measurement.eq(timer),
                         timer.eq(0)]
        with m.Else():
            m.d.sync += timer.eq(timer+1)

        off = Signal()
        duty = Signal(range(max_delay))

        # Duty cycle
        # Regulates power to the motor by turning it off
        # if larger than duty cycle
        with m.If(duty < max_delay):
            m.d.sync += duty.eq(0)
        with m.Else():
            m.d.sync += duty.eq(duty+1)

        # Motor On / Off
        with m.If(duty < delay):
            m.d.sync += off.eq(1)
        with m.Else():
            m.d.sync += off.eq(0)

        # https://www.mathworks.com/help/mcb/ref/sixstepcommutation.html
        with m.If((~self.enable_prism) | off):
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(statefilter == 1):  # V --> W, 001
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(1),
                         self.wL.eq(1),
                         self.wH.eq(0)]
        with m.Elif(statefilter == 3):  # V --> U, 011
            m.d.comb += [self.uL.eq(1),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(1),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(statefilter == 2):  # W --> U, 010
            m.d.comb += [self.uL.eq(1),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(1)]
        with m.Elif(statefilter == 6):  # W --> V, 110
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(1),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(1)]
        with m.Elif(statefilter == 4):  # U --> V, 100
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(1),
                         self.vL.eq(1),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(statefilter == 5):  # U --> W, 101
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(1),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(1),
                         self.wH.eq(0)]
        # with m.Else():         # disabled
        #     m.d.comb += [bldc.uL.eq(0),
        #                  bldc.uH.eq(0),
        #                  bldc.vL.eq(0),
        #                  bldc.vH.eq(0),
        #                  bldc.wL.eq(0),
        #                  bldc.wH.eq(0)]
        return m


class TestBLDC(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Driver
    FRAGMENT_ARGUMENTS = {'platform': platform}

    @sync_test_case
    def test_sensor(self):
        # TODO: add some tests
        yield
        # dut = self.dut
        # yield
        # self.assertEqual((yield dut.sensor), 0)
        # (yield dut.platform.bldc.sensor.eq(1))
        # yield
        # self.assertEqual((yield dut.sensor), 1)


if __name__ == "__main__":
    unittest.main()
