""" BLDC board provided on seperate board see 
    https://hackaday.io/project/21933-prism-laser-scanner/log/201104-prism-motor-bugeja-style
"""
import itertools
import unittest

from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.build import ResourceError
from platforms import TestPlatform
from luna.gateware.interface.spi import SPICommandInterface, SPIBus


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor

       frequency in Hz, rotation frequency of BLDC motor
       state of the motor, wether motor is turned on
    '''

    def __init__(self, platform,
                 top=False):
        """
        platform  -- pass test platform
        top       -- True if top module
        """
        self.platform = platform
        self.top = top
        self.spi = SPIBus()

    def elaborate(self, platform):
        m = Module()
        state = Signal(3)

        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        if platform and self.top:
            board_spi = platform.request("debug_spi")
            spi2 = synchronize(m, board_spi)

            leds = [res.o for res in get_all_resources("led")]
            bldc = platform.request("bldc")
            m.d.comb += self.spi.connect(spi2)

            m.d.comb += [leds[0].eq(bldc.sensor0),
                         leds[1].eq(bldc.sensor1),
                         leds[2].eq(bldc.sensor2)]
            # tested by trying out all possibilities
            m.d.sync += state.eq(Cat(bldc.sensor0,
                                     bldc.sensor1,
                                     bldc.sensor2))
        else:
            platform = self.platform
            bldc = platform.bldc

        target = 200000
        max_measurement = target*10
        measurement = Signal(range(max_measurement))
        timer = Signal().like(measurement)
        statefilter = Signal(3)
        stateold = Signal(3)
        max_delay = 1000
        delay = Signal(range(max_delay))

        # Statefilter
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
        with m.If(timer >= max_measurement-1):
            m.d.sync += [timer.eq(0),
                         measurement.eq(timer)]
        with m.Elif((statefilter == 1) & (stateold != 1)):
            m.d.sync += [measurement.eq(timer),
                         timer.eq(0)]
        with m.Else():
            m.d.sync += timer.eq(timer+1)

        spi = self.spi
        interf = SPICommandInterface(command_size=1*8,
                                     word_size=4*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf
        m.d.sync += interf.word_to_send.eq(measurement)

        off = Signal()
        duty = Signal(range(max_delay))

        # Duty timer
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
        with m.If(off):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(statefilter == 1):  # V --> W, 001
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
        with m.Elif(statefilter == 3):  # V --> U, 011
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(statefilter == 2):  # W --> U, 010
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(statefilter == 6):  # W --> V, 110
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(statefilter == 4):  # U --> V, 100
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(statefilter == 5):  # U --> W, 101
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
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
    FRAGMENT_ARGUMENTS = {'frequency': 3,
                          'platform': platform}

    @sync_test_case
    def test_sensor(self):
        '''counts steps in accounting for direction'''
        dut = self.dut
        yield
        self.assertEqual((yield dut.sensor), 0)
        (yield dut.platform.bldc.sensor.eq(1))
        yield
        self.assertEqual((yield dut.sensor), 1)


if __name__ == "__main__":
    unittest.main()
