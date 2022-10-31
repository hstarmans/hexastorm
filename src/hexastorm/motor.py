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
        hallstate = Signal(3)
        motorstate = Signal().like(hallstate)

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
        # hall debug
        # for idx in range(len(self.leds)):
        #    m.d.comb += self.leds[idx].eq(self.hall[idx])

        # tested by trying out all possibilities
        m.d.sync += hallstate.eq(Cat(~self.hall[0],
                                     ~self.hall[1],
                                     ~self.hall[2]))
       
        def get_cycletime(frequency):
            return int(platform.laser_var['CRYSTAL_HZ']/
                       (frequency*12))

        start_frequency = 1
        start_statetime = get_cycletime(start_frequency)
        rotationtime = Signal(range(int(start_statetime*15+1)))
        hallpulsecntr = Signal.like(rotationtime)
        mtrpulsecntr = Signal(range(int(start_statetime+1)))
        rotating = Signal()
        transitions_cycle = 12
        statecounterhall = Signal(range(transitions_cycle+1))

        with m.FSM(reset='ROTATION', name='algo'):
            with m.State('ROTATION'):
                with m.If(mtrpulsecntr == start_statetime):
                    m.d.sync += mtrpulsecntr.eq(0)
                    with m.If(motorstate == 5):
                        m.d.sync += motorstate.eq(0)
                    with m.Else():
                        m.d.sync += motorstate.eq(motorstate+1)
                with m.Else():
                    m.d.sync += mtrpulsecntr.eq(mtrpulsecntr+1)
                with m.If(rotating == 1):
                    m.next = 'HALL'
            with m.State('HALL'):
                with m.If(hallstate == 1):
                    m.d.sync += motorstate.eq(0)
                with m.Elif(hallstate == 2):
                    m.d.sync += motorstate.eq(2)
                with m.Elif(hallstate == 3):
                    m.d.sync += motorstate.eq(1)
                with m.Elif(hallstate == 4):
                    m.d.sync += motorstate.eq(4)
                with m.Elif(hallstate == 5):
                    m.d.sync += motorstate.eq(5)
                with m.Elif(hallstate == 6):
                    m.d.sync += motorstate.eq(3)
                # leds off and all leds on is filtered
                with m.If(rotating == 0):
                    m.next = 'ROTATION'

        statecounter = Signal(range(transitions_cycle+1))
        stateold = Signal.like(hallstate)
        stateoldold = Signal().like(hallstate)
        statesuperold = Signal().like(hallstate)
        
        m.d.comb += self.leds[0].eq(rotating)
       
        # max_delay = 1000
        # delay = Signal(range(max_delay))
        # target = 200000
        # max_measurement = target*10
        # measurement = Signal(range(max_measurement))

        # PID controller
        # step = max_delay//100
        # with m.If((measurement > target) & (delay >= step)):
        #     m.d.sync += delay.eq(delay - step)
        # with m.Elif((measurement < target) & (delay < (max_delay-step))):
        #     m.d.sync += delay.eq(delay+step)
        # with m.Else():
        #     m.d.sync += delay.eq(0)

        # Measure Cycle
        # Measures the speed of the motor by checking
        # how often state 1 is reached
        with m.If((hallstate != stateold) &
                  (hallstate != stateoldold) &
                  (hallstate != statesuperold)):
            m.d.sync += [stateold.eq(hallstate),
                         stateoldold.eq(stateold),
                         statesuperold.eq(stateoldold)]
            with m.If(statecounter > transitions_cycle):
                m.d.sync += [statecounter.eq(0),
                             hallpulsecntr.eq(0)]
                # store measurerement
                with m.If(hallpulsecntr > (get_cycletime(100)*12)):
                    # rotating so accelerate
                    m.d.sync += [rotationtime.eq(hallpulsecntr),
                                 rotating.eq(1)]
                with m.Else():
                    # not rotating
                    m.d.sync += [rotationtime.eq(0),
                                 rotating.eq(0)]
            with m.Else():
                m.d.sync += statecounter.eq(statecounter+1)
        with m.Else():
            with m.If(hallpulsecntr >= int(start_statetime*14)):
                m.d.sync += self.leds[1].eq(~self.leds[1])
                m.d.sync += [hallpulsecntr.eq(0),
                             rotating.eq(0),
                             rotationtime.eq(0)]
            with m.Else():
                m.d.sync += hallpulsecntr.eq(hallpulsecntr+1)

        off = Signal()

#         duty = Signal(range(max_delay))

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
        with m.If((~self.enable_prism)):# | off):
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(motorstate == 1):  # V --> W, 001 BLUE
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(1),
                         self.wL.eq(1),
                         self.wH.eq(0)]
        with m.Elif(motorstate == 3):  # V --> U, 011 CYAN
            m.d.comb += [self.uL.eq(1),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(1),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(motorstate == 2):  # W --> U, 010 GREEN
            m.d.comb += [self.uL.eq(1),
                         self.uH.eq(0),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(1)]
        with m.Elif(motorstate == 6):  # W --> V, 101 MAGENTA
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(0),
                         self.vL.eq(1),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(1)]
        with m.Elif(motorstate == 4):  # U --> V, 100 RED
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(1),
                         self.vL.eq(1),
                         self.vH.eq(0),
                         self.wL.eq(0),
                         self.wH.eq(0)]
        with m.Elif(motorstate == 5):  # U --> W, 110 YELLOW
            m.d.comb += [self.uL.eq(0),
                         self.uH.eq(1),
                         self.vL.eq(0),
                         self.vH.eq(0),
                         self.wL.eq(1),
                         self.wH.eq(0)]
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
