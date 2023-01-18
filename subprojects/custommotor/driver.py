""" BLDC board provided on separate board see,
    https://hackaday.io/project/21933-prism-laser-scanner/log/201104-prism-motor-bugeja-style
"""
import itertools
import unittest

from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.build import ResourceError

from platforms import TestPlatform


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor
    '''

    def __init__(self, platform, word='cycletime', divider=800,
                 top=False):
        """
        platform  -- pass test platform
        top       -- True if top module
        mode      -- defines what is sent back
        """
        self.platform = platform
        self.top = top
        self.spi = SPIBus()
        self.word = word
        self.divider = divider

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(8)
        hallstate = Signal().like(motorstate)
        hallfilter = Signal().like(hallstate)
        # total number of states in cycle
        states_fullcycle = 12
        # clock is downscaled to filter out
        # noise coming to the hall sensors

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
            '''time needed for one state'''
            return int((platform.laser_var['CRYSTAL_HZ'])/
                       (frequency*states_fullcycle))

        #slow = ClockDomain("slow")
        #m.domains += slow
        divider_cnt = Signal(range(self.divider//2))

        # hall filter
        with m.If(divider_cnt == self.divider//2-1):
            m.d.sync += [divider_cnt.eq(0),
                         # noise filter
                         hallfilter.eq(hallstate)]
                         #slow.clk.eq(~slow.clk)]
        with m.Else():
            m.d.sync += divider_cnt.eq(divider_cnt+1)

        # You need to test this first with flashing a led
        start_freq = 2  # Hz
        start_statetime = get_statetime(start_freq)
        # range(int(start_statetime*transitions_cycle))
        rotationtime = Signal(24)
        hallcntr = Signal.like(rotationtime)
        cycletime = Signal.like(rotationtime)
        mtrpulsecntr = Signal(range(int((start_statetime+1))))
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
                with m.If(hallfilter == 1):
                    m.d.sync += motorstate.eq(1)
                with m.Elif(hallfilter == 2):
                    m.d.sync += motorstate.eq(3)
                with m.Elif(hallfilter == 3):
                    m.d.sync += motorstate.eq(2)
                with m.Elif(hallfilter == 4):
                    m.d.sync += motorstate.eq(5)
                with m.Elif(hallfilter == 5):
                    m.d.sync += motorstate.eq(6)
                with m.Elif(hallfilter == 6):
                    m.d.sync += motorstate.eq(4)
                # hall off and all hall on are filtered
                # i.e. cases 0 en 7
                with m.If(rotating == 0):
                    m.next = 'ROTATION'

        statecounter = Signal(range(states_fullcycle//2))
        stateold = Signal.like(hallstate)
        countsperdegree = Signal.like(cycletime)

        # Cycle time measurement
        with m.If((hallfilter != stateold) &
                  (hallfilter > 0) &
                  (hallfilter < 7)):
            m.d.sync += stateold.eq(hallfilter)
            with m.If(statecounter == states_fullcycle//2-1):
                # TODO: add filter to remove invalid cycle times, which are too small
                m.d.sync += [statecounter.eq(0),
                             hallcntr.eq(0),
                             countsperdegree.eq((hallcntr>>8)+(hallcntr>>10)+(hallcntr>>11)+(hallcntr>>13)+(hallcntr>>14)),]
                             #cycletime.eq(cycletime-(cycletime>>2)+(hallcntr>>2))]
                             #cycletime.eq(hallcntr)]
                # store measurerement
                # only if the measurement is reasonable
                # if there are too few ticks --> not reasonable
                with m.If(hallcntr < (get_statetime(start_freq)*states_fullcycle)):
                    # rotating so use hall sensors
                    m.d.sync += rotating.eq(1)
            with m.Else():
                m.d.sync += statecounter.eq(statecounter+1)
        # counter is overflowing, implying there is no rotation
        with m.Elif(hallcntr == int(start_statetime*states_fullcycle)):
            #m.d.sync += leds[1].eq(~leds[1])
            m.d.sync += [hallcntr.eq(0),
                         rotating.eq(0),
                         rotationtime.eq(0)]
        with m.Else():
            m.d.sync += hallcntr.eq(hallcntr+1)

        # degree counter
        degree_cnt = Signal().like(countsperdegree)
        angle = Signal(range(180))
        with m.If((countsperdegree > 500) & (countsperdegree < 5000)):
            # Assumption 1:
            #         - just running a counter should be uniform
            #               seems to be the case --> really gets to it after two minutes
            #                with m.If((hallfilter == 3) & (stateold == 1)):
            #                m.d.sync += angle.eq(0)
            #         - run a counter after each cycle
            #               (hallfilter == 3) & (stateold == 1)
            #               does work
            #         - run a counter using the hall triger
            #              you made a mistake, states are ordered differently
            with m.If((hallfilter != stateold) &
                      (hallfilter > 0) &
                      (hallfilter < 7)):
                m.d.sync += degree_cnt.eq(0)
                # mapping!! between motorstate
                #           and hallfilter is not
                #           trivial
                with m.If(hallfilter == 1):
                    m.d.sync += angle.eq(0)
                with m.Elif(hallfilter == 3):
                    m.d.sync += angle.eq(33)
                with m.Elif(hallfilter == 2):
                    m.d.sync += angle.eq(63)
                with m.Elif(hallfilter == 6):
                    m.d.sync += angle.eq(100)
                with m.Elif(hallfilter == 4):
                    m.d.sync += angle.eq(108)
                with m.Elif(hallfilter == 5):
                    m.d.sync += angle.eq(162)
            # countsperdegree varies, so I do not use
            #  degree_cnt == countsperdegree-1

            with m.Elif((degree_cnt+2) > countsperdegree):
                m.d.sync += degree_cnt.eq(0)
                with m.If(angle == 180-1):
                    m.d.sync += angle.eq(0)
                with m.Else():
                    m.d.sync += angle.eq(angle+1)
            with m.Else():
                m.d.sync += degree_cnt.eq(degree_cnt+1)


        spi = self.spi
        interf = SPICommandInterface(command_size=1*8,
                                     word_size=4*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf
        with m.If(motorstate != 0):
            if self.word == 'hallfilter':
                m.d.sync += interf.word_to_send.eq(hallfilter)
            elif self.word == 'cycletime':
                m.d.sync += interf.word_to_send.eq(cycletime)
            elif self.word == 'statecounter':
                m.d.sync += interf.word_to_send.eq(statecounter)
            elif self.word == 'anglecounter':
                m.d.sync += interf.word_to_send.eq(countsperdegree)
            elif self.word == 'angle':
                m.d.sync += interf.word_to_send.eq(angle)
            else:
                raise Exception(f"{motorstate} not supported")
        with m.Else():
            m.d.sync += interf.word_to_send.eq(7)

        # PID controller
        # Ideal is controlling both voltage and current
        # Speed
        off = Signal()
        # # RPM --> ticks
        # target = int(round(13.56E6/(2000/60)))
        # max_delay = 1000
        # duty_step = max_delay//100
        # # specifies current delay (counter)
        # delay = Signal(range(max_delay))
        # # specificies where you are in duty cycle (counter)
        # duty = Signal(range(max_delay))
        # step = 1 #max_delay//100
        # with m.If(rotating == 0):
        #     m.d.sync += delay.eq(0)
        # # CASE: rotating too slow
        # with m.Elif((rotationtime > target) & (delay >= step)):
        #     m.d.sync += delay.eq(delay - step)
        # # CASE: rotating too fast
        # with m.Elif((rotationtime < target) & (delay < (max_delay-step)) & (delay<int(0.95*max_delay))):
        #     m.d.sync += delay.eq(delay+step)

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
