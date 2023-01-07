""" BLDC board provided on seperate board see 
    https://hackaday.io/project/21933-prism-laser-scanner/log/201104-prism-motor-bugeja-style
"""
import itertools
import unittest

from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.build import ResourceError
from amaranth.hdl.mem import Array

from platforms import TestPlatform


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor
    '''

    def __init__(self, platform, top=False):
        """
        platform  -- pass test platform
        top       -- True if top module
        """
        self.platform = platform
        self.top = top
        self.spi = SPIBus()

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(8)
        hallstate = Signal().like(motorstate)
        transitions_cycle = 12 # total number of states in cycle

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
            m.d.comb += hallstate.eq(Cat(~bldc.sensor0,
                                         ~bldc.sensor1,
                                         ~bldc.sensor2))
            m.d.comb += leds[0].eq(hallstate[0])
            m.d.comb += leds[1].eq(hallstate[1])
            m.d.comb += leds[2].eq(hallstate[2])
        else:
            platform = self.platform
            bldc = platform.bldc

        def get_statetime(frequency):
            '''time needed for one state

            it is assumed there are 12 states in one full cycle
            '''
            return int(platform.laser_var['CRYSTAL_HZ']/(frequency*transitions_cycle))

        start_frequency = 2  # Hz

        start_statetime = get_statetime(start_frequency)
        # range(int(start_statetime*transitions_cycle))
        rotationtime = Signal(24)
        hallpulsecntr = Signal.like(rotationtime)
        statetime = Signal.like(Cat(hallstate, rotationtime))
        mtrpulsecntr = Signal(range(int(start_statetime+1)))
        rotating = Signal()

        with m.FSM(reset='ROTATION', name='algo'):
            with m.State('ROTATION'):
                with m.If(mtrpulsecntr == start_statetime):
                    m.d.sync += mtrpulsecntr.eq(0)
                    with m.If(motorstate == 6):
                        m.d.sync += motorstate.eq(1)
                    with m.Else():
                        m.d.sync += motorstate.eq(motorstate+1)
                with m.Else():
                    m.d.sync += mtrpulsecntr.eq(mtrpulsecntr+1)
                with m.If(rotating == 1):
                    m.next = 'HALL'
            with m.State('HALL'):
                with m.If(hallstate == 1):
                    m.d.sync += motorstate.eq(1)
                with m.Elif(hallstate == 2):
                    m.d.sync += motorstate.eq(3)
                with m.Elif(hallstate == 3):
                    m.d.sync += motorstate.eq(2)
                with m.Elif(hallstate == 4):
                    m.d.sync += motorstate.eq(5)
                with m.Elif(hallstate == 5):
                    m.d.sync += motorstate.eq(6)
                with m.Elif(hallstate == 6):
                    m.d.sync += motorstate.eq(4)
                # hall off and all hall on are filtered
                # i.e. cases 0 en 7
                with m.If(rotating == 0):
                    m.next = 'ROTATION'

        statecounter = Signal(range(transitions_cycle+1))
        stateold = Signal.like(hallstate)
        stateoldold = Signal().like(hallstate)
        statesuperold = Signal().like(hallstate)

        # Cycle time measurement
        with m.If((hallstate != stateold) &
                  (hallstate != stateoldold) &
                  (hallstate != statesuperold) &
                  (hallstate > 0) &
                  (hallstate < 7)):
            m.d.sync += [# timers
                         hallpulsecntr.eq(0),
                         rotationtime.eq(hallpulsecntr+rotationtime),
                         statetime.eq(Cat(hallpulsecntr, motorstate)),
                         # states
                         stateold.eq(hallstate),
                         stateoldold.eq(stateold),
                         statesuperold.eq(stateoldold)]
            with m.If(statecounter == transitions_cycle):
                m.d.sync += [statecounter.eq(0),
                             rotationtime.eq(0)]
                # store measurerement
                # only if the measurement is reasonable
                # if there are too few ticks --> not reasonable
                with m.If(rotationtime > (get_statetime(100)*transitions_cycle)):
                    # rotating so use hall sensors
                    m.d.sync += rotating.eq(1)
                # not sure what to do with unreasonable ticks
                # if speed is too high it is ignored
            with m.Else():
                m.d.sync += statecounter.eq(statecounter+1)
        with m.Else():
            # there should already have been a reset
            # due to statecounter is 12
            # ergo you r not rotating
            with m.If(hallpulsecntr == int(start_statetime*12)):
                #m.d.sync += leds[1].eq(~leds[1])
                m.d.sync += [hallpulsecntr.eq(0),
                             rotating.eq(0),
                             rotationtime.eq(0)]
            with m.Else():
                m.d.sync += hallpulsecntr.eq(hallpulsecntr+1)

        spi = self.spi
        interf = SPICommandInterface(command_size=1*8,
                                     word_size=4*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf
        with m.If(motorstate != 0):
            m.d.sync += interf.word_to_send.eq(motorstate)
        with m.Else():
            m.d.sync += interf.word_to_send.eq(7)

        # PID controller
        # Ideal is controlling both voltage and current
        # Speed
        off = Signal()
#         # RPM --> ticks
#         target = int(round(13.56E6/(2000/60)))
#         max_delay = 1000
#         duty_step = max_delay//100
#         # specifies current delay (counter)
#         delay = Signal(range(max_delay))
#         # specificies where you are in duty cycle (counter)
#         duty = Signal(range(max_delay))
#         step = 1 #max_delay//100
#         with m.If(rotating == 0):
#             m.d.sync += delay.eq(0)
#         # CASE: rotating too slow
#         with m.Elif((rotationtime > target) & (delay >= step)):
#             m.d.sync += delay.eq(delay - step)
#         # CASE: rotating too fast
#         with m.Elif((rotationtime < target) & (delay < (max_delay-step)) & (delay<int(0.95*max_delay))):
#             m.d.sync += delay.eq(delay+step)

#         # Duty timer
#         with m.If(duty < max_delay):
#             m.d.sync += duty.eq(0)
#         with m.Else():
#             m.d.sync += duty.eq(duty+1)

#         # Motor On / Off
#         with m.If(duty < delay):
#             m.d.sync += off.eq(1)
#         with m.Else():
#             m.d.sync += off.eq(0)

        # https://www.mathworks.com/help/mcb/ref/sixstepcommutation.html
        with m.If(off & (~rotating)):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 1):  # V --> W, 001
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 2):  # V --> U, 011
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 3):  # W --> U, 010
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(motorstate == 4):  # W --> V, 110
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(motorstate == 5):  # U --> V, 100
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 6):  # U --> W, 101
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
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
