import itertools
import unittest

from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from amaranth import Elaboratable, Module, Signal, Cat, signed
from amaranth.build import ResourceError
import numpy as np

from platforms import TestPlatform


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor
    '''

    def __init__(self,
                 platform,
                 word='PIcontrol',
                 divider=800,
                 PIcontrol=False,
                 top=False):
        """
        platform  -- pass test platform
        top       -- True if top module
        word      -- defines what word is sent back via SPI
        divider   -- clock is downsampled to retrieve Hall
        PIControl -- enabled Position Integral control
        """
        self.platform = platform
        self.top = top
        self.spi = SPIBus()
        self.word = word
        self.divider = divider
        self.PIcontrol = PIcontrol

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(range(8))
        hallstate = Signal().like(motorstate)
        hallfilter = Signal().like(hallstate)
        # total number of states in 360 degrees
        # state 3 does not propagate!!, so not 12
        states_fullcycle = 10
        angle = Signal(range(180))
        
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
            m.d.comb += [self.spi.connect(spi2),
                         hallstate.eq(Cat(~bldc.sensor0,
                                          ~bldc.sensor1,
                                          ~bldc.sensor2)),
                         leds[0].eq(hallstate[0]),
                         leds[1].eq(hallstate[1]),
                         leds[2].eq(hallstate[2])]
        else:
            platform = self.platform
            bldc = platform.bldc

        def get_statetime(frequency):
            '''time needed for one state'''
            return int((platform.laser_var['CRYSTAL_HZ'])/
                       (frequency*states_fullcycle))

        # angle counter limits
        lowerlimit = int((get_statetime(100)*states_fullcycle/360))
        upperlimit = int((get_statetime(10)*states_fullcycle/360))

        # add a clock as follows
        # slow = ClockDomain("slow")
        # m.domains += slow
        # drive this from m.d.sync
        # slow.clk.eq(~slow.clk)
        divider_cnt = Signal(range(self.divider//2))

        # clock is downscaled to filter out
        # noise coming to the hall sensors
        # this creates hall filter
        with m.If(divider_cnt == self.divider//2-1):
            m.d.sync += [divider_cnt.eq(0),
                         hallfilter.eq(hallstate)]
        with m.Else():
            m.d.sync += divider_cnt.eq(divider_cnt+1)
        
        start_freq = 2  # Hz
        start_statetime = get_statetime(start_freq)
        # range(int(start_statetime*transitions_cycle))
        rotationtime = Signal(24)
        hallcntr = Signal.like(rotationtime)
        cycletime = Signal.like(rotationtime)
        mtrpulsecntr = Signal(range(int((start_statetime+1))))
        # countsperdegree = Signal.like(cycletime)
        countsperdegree = Signal(16)
        rotating = Signal()
        with m.FSM(reset='ROTATION', name='algo'):
            # rotation is clock wise
            with m.State('ROTATION'):
                with m.If(mtrpulsecntr == start_statetime):
                    m.d.sync += mtrpulsecntr.eq(0)
                    with m.If(motorstate == 1):
                        m.d.sync += motorstate.eq(6)
                    with m.Elif(motorstate == 0):
                        m.d.sync += motorstate.eq(1)
                    with m.Else():
                        m.d.sync += motorstate.eq(motorstate-1)
                with m.Else():
                    m.d.sync += mtrpulsecntr.eq(mtrpulsecntr+1)
                with m.If(rotating == 1):
                    m.next = 'HALL'
            # rotation is clockwise
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
                with m.Elif((countsperdegree > lowerlimit)
                            & (countsperdegree < upperlimit)
                            & (self.word != 'hallfilter')):
                    m.next = 'IMPHALL'
            with m.State('IMPHALL'):
                with m.If(angle < 30):
                    m.d.sync += motorstate.eq(1)
                with m.Elif(angle < 60):
                    m.d.sync += motorstate.eq(2)
                with m.Elif(angle < 90):
                    m.d.sync += motorstate.eq(3)
                with m.Elif(angle < 120):
                    m.d.sync += motorstate.eq(4)
                with m.Elif(angle < 150):
                    m.d.sync += motorstate.eq(5)
                with m.Elif(angle < 181):
                    m.d.sync += motorstate.eq(6)
                with m.If(rotating == 0):
                    m.next = 'ROTATION'

        statecounter = Signal(range(states_fullcycle//2))
        stateold = Signal.like(hallstate)

        clock = int(platform.clks[platform.hfosc_div]*1E6)
        RPM = platform.laser_var['RPM']
        # ticks required for 6 states
        setpoint = int(round((clock/(2*RPM/60)))/180)

        # Cycle time measurement
        with m.If((hallfilter != stateold) &
                  (hallfilter != 3) &  # due to sensor issue 3 is barely triggered
                  (hallfilter > 0) &
                  (hallfilter < 7)):
            m.d.sync += stateold.eq(hallfilter)
            with m.If(statecounter == states_fullcycle//2-1):
                # there used to be filter which removed invalid times
                # measurements seems good enough for no filter
                m.d.sync += [statecounter.eq(0),
                             hallcntr.eq(0),
                             countsperdegree.eq((hallcntr >> 8) + 
                                                (hallcntr >> 10) +
                                                (hallcntr >> 11) + 
                                                (hallcntr >> 13) +
                                                (hallcntr >> 14)),
                             cycletime.eq(hallcntr)]
                # store measurerement
                # only if the measurement is reasonable
                # if there are too few ticks --> not reasonable
                with m.If(hallcntr < (get_statetime(start_freq)
                                      * states_fullcycle)):
                    # rotating so use hall sensors
                    m.d.sync += rotating.eq(1)
            with m.Else():
                m.d.sync += statecounter.eq(statecounter+1)
        # counter is overflowing, implying there is no rotation
        with m.Elif(hallcntr == int(start_statetime*states_fullcycle)):
            m.d.sync += [hallcntr.eq(0),
                         rotating.eq(0),
                         rotationtime.eq(0)]
        with m.Else():
            m.d.sync += hallcntr.eq(hallcntr+1)

        # degree counter
        degree_cnt = Signal().like(countsperdegree)

        # beta probably negative due to
        # motor response time, likely speed dependent
        beta = 0 
        hall_degs = (np.array([0, 42, 42, 91, 118, 142])+beta+180) % 180
        # combination
        hall_degs = hall_degs.tolist()

        # Assumption 1:
        # - when to switch from hall feedback --> control algo
        # - alter between beta's
        with m.If((hallfilter != stateold) &
                  (hallfilter != 3) &
                  (hallfilter > 0) &
                  (hallfilter < 7)):
            m.d.sync += degree_cnt.eq(0)
            # mapping!! between motorstate
            #           and hallfilter is not
            #           trivial
            with m.If(hallfilter == 1):
                m.d.sync += angle.eq(hall_degs[0])
            with m.Elif(hallfilter == 3):
                m.d.sync += angle.eq(hall_degs[1])
            with m.Elif(hallfilter == 2):
                m.d.sync += angle.eq(hall_degs[2])
            with m.Elif(hallfilter == 6):
                m.d.sync += angle.eq(hall_degs[3])
            with m.Elif(hallfilter == 4):
                m.d.sync += angle.eq(hall_degs[4])
            with m.Elif(hallfilter == 5):
                m.d.sync += angle.eq(hall_degs[5])
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

        # Current controlled via duty cycle
        max_delay = 100_000
        #max_delaylimit = -(setpoint-upperlimit) >> 2
        #min_delaylimit = -(setpoint-lowerlimit) >> 2
        #delay = Signal(range(min_delaylimit,
        #                     max_delaylimit))
        delay = Signal(signed(32))

        # PID controller
        assert ((upperlimit > lowerlimit) &
                (setpoint > 0) &
                (lowerlimit > 0))
        lower_l = setpoint-upperlimit
        upper_l = setpoint-lowerlimit
        int_lower_l = lower_l*1000
        int_upper_l = upper_l*1000
        err = Signal(range(lower_l,
                           upper_l))
        assert ((upper_l > 0) & (lower_l < 0))
        der = Signal(range(lower_l-upper_l,
                           upper_l-lower_l))
        intg = Signal(range(int_lower_l,
                            int_upper_l))
        

        with m.If(rotating == 0):
            m.d.sync += [delay.eq(max_delay),
                         err.eq(0),
                         intg.eq(0)]
        with m.Elif((countsperdegree > lowerlimit)
                  & (countsperdegree < upperlimit)):
            # ugly use state machine!
            with m.If(hallcntr == 0):
                m.d.sync += [err.eq(setpoint-countsperdegree),
                             der.eq(err-setpoint+countsperdegree),
                             # it is assumed the integral "does not blow up"
                             # you can monitor it via SPI
                             intg.eq(intg+err)]
            with m.Else():
                # bitshifts used to avoid multiplications
                m.d.sync += delay.eq(-(err >> 5)- (intg >> 11))

        off = Signal()
        duty = Signal(range(max_delay))

        # Duty timer
        with m.If(duty < max_delay):
            m.d.sync += duty.eq(0)
        with m.Else():
            m.d.sync += duty.eq(duty+1)

        # Motor On / Off
        with m.If(duty < delay):
            m.d.sync += off.eq(0)
        with m.Else():
            m.d.sync += off.eq(1)

        # https://www.mathworks.com/help/mcb/ref/sixstepcommutation.html
        with m.If((off & self.PIcontrol) & rotating):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 2):  # V --> W, 001
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 3):  # V --> U, 011
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 4):  # W --> U, 010
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(motorstate == 5):  # W --> V, 110
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(motorstate == 6):  # U --> V, 100
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(motorstate == 1):  # U --> W, 101
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]

        # depending on mode, a certain word is sent back
        spi = self.spi
        interf = SPICommandInterface(command_size=1*8,
                                     word_size=4*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf

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
        elif self.word == 'PIcontrol':
            m.d.sync += interf.word_to_send.eq(Cat(countsperdegree,
                                                   delay))
        else:
            raise Exception(f"{motorstate} not supported")

        return m


class TestBLDC(LunaGatewareTestCase):
    '''place holder for tests...
       there are no tests at the moment!
    '''
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
