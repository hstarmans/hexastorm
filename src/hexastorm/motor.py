import unittest

from luna.gateware.interface.spi import SPICommandInterface
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from amaranth import Elaboratable, Module, Signal, Cat, signed
from amaranth.hdl.mem import Array
import numpy as np

from .platforms import TestPlatform
from .resources import get_all_resources
from .constants import WORD_BYTES


class Driver(Elaboratable):
    '''Drives 3 pole BLDC motor

       Motor is is driven via a six step commutation cycle.
       Driver starts in a forced mode. It then starts to rely
       on the hall sensors and speeds up. It calculates the speed
       and uses an interpolation table to improve the triggering
       of the motor windings.
       Finally, position integral control is used to stabilize
       the rotor speed.

       I: enable_prism   -- enable pin scanner motor
       I: Hall (3)       -- hall sensor measure state motor
       O: leds (3)       -- display measurements Hall sensors
       O: bldc (6)       -- powers BLDC motor
       O: debugmotor     -- used to collect info on the motor driver
    '''
    def __init__(self,
                 platform,
                 top=False,
                 divider=800,
                 PIcontrol=True):
        """
        platform  -- pass test platform
        top       -- True if top module
        divider   -- Hall sensors are sampled at lower rate to
                     remove noise
        PIControl -- enabled Position Integral control
                     RPM set via laser_var on platform
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
        self.debugword = Signal(WORD_BYTES*8)
        # depending on mode, a certain word is sent back
        self.mode = platform.laser_var['MOTORDEBUG']
        self.divider = divider
        self.PIcontrol = PIcontrol

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(8)
        hallstate = Signal().like(motorstate)
        hallfilter = Signal().like(hallstate)
        # total number of states in 360 degrees
        states_fullcycle = 12
        angle = Signal(range(180))
        # angle counter limits
        lowerlimit = 500
        upperlimit = 5000
        mode = self.mode

        uL, uH, vL, vH, wL, wH = (self.uL, self.uH, self.vL, self.vH,
                                  self.wL, self.wH)

        if platform and self.top:
            leds = [res.o for res in get_all_resources(platform, "led")]
            for idx in range(len(self.leds)):
                m.d.comb += self.leds[idx].eq(leds[idx])
            bldc = platform.request("bldc")
            m.d.comb += self.enable_prism.eq(1)
            m.d.comb += [bldc.uL.eq(uL),
                         bldc.uH.eq(uH),
                         bldc.vL.eq(vL),
                         bldc.vH.eq(vH),
                         bldc.wL.eq(wL),
                         bldc.wH.eq(wH)]
            m.d.comb += [hallstate.eq(Cat(~bldc.sensor0,
                                          ~bldc.sensor1,
                                          ~bldc.sensor2))]

            spi = self.spi
            interf = SPICommandInterface(command_size=1*8,
                                         word_size=4*8)
            m.d.comb += interf.spi.connect(spi)
            m.submodules.interf = interf

        # hall debug
        for idx in range(len(self.leds)):
            m.d.comb += self.leds[idx].eq(self.hall[idx])

        # tested by trying out all possibilities
        m.d.sync += hallstate.eq(Cat(~self.hall[0],
                                     ~self.hall[1],
                                     ~self.hall[2]))

        def get_statetime(frequency):
            '''time needed for one state'''
            return int((platform.laser_var['CRYSTAL_HZ'])/
                       (frequency*states_fullcycle))

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
                with m.Elif((countsperdegree > lowerlimit)
                            & (countsperdegree < upperlimit)
                            & (mode != 'hallfilter')):
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
                  (hallfilter > 0) &
                  (hallfilter < 7)):
            m.d.sync += stateold.eq(hallfilter)
            with m.If(statecounter == states_fullcycle//2-1):
                # TODO: add filter to remove invalid cycle times, 
                #       which are too small
                m.d.sync += [statecounter.eq(0),
                             hallcntr.eq(0),
                             # division by 180 via bitshifts
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
        beta = -15
        hall_degs = (np.array([0, 33, 63, 100, 108, 162])+beta+180) % 180
        # combination
        hall_degs = hall_degs.tolist()

        # in the end an angle is produced which should better approximate
        # the actual position of the rotor
        with m.If((countsperdegree > lowerlimit)
                  & (countsperdegree < upperlimit)):
            # Assumption 1:
            # - when to switch from hall feedback --> control algo
            # - alter between beta's
            with m.If((hallfilter != stateold) &
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
        max_delay = 1000
        # width fixed at 16 bits to simplify communication
        # max_delaylimit = -(setpoint-upperlimit) >> 2
        # min_delaylimit = -(setpoint-lowerlimit) >> 2
        # delay = Signal(range(min_delaylimit,
        #                      max_delaylimit))
        delay = Signal(signed(16))

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
                m.d.sync += delay.eq(-(err >> 3) - (intg >> 10))

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
        with m.If(((off & self.PIcontrol) & rotating) | (~self.enable_prism)):
            m.d.comb += [uL.eq(0),
                         uH.eq(0),
                         vL.eq(0),
                         vH.eq(0),
                         wL.eq(0),
                         wH.eq(0)]
        with m.Elif(motorstate == 1):  # V --> W, 001
            m.d.comb += [uL.eq(0),
                         uH.eq(0),
                         vL.eq(0),
                         vH.eq(1),
                         wL.eq(1),
                         wH.eq(0)]
        with m.Elif(motorstate == 2):  # V --> U, 011
            m.d.comb += [uL.eq(1),
                         uH.eq(0),
                         vL.eq(0),
                         vH.eq(1),
                         wL.eq(0),
                         wH.eq(0)]
        with m.Elif(motorstate == 3):  # W --> U, 010
            m.d.comb += [uL.eq(1),
                         uH.eq(0),
                         vL.eq(0),
                         vH.eq(0),
                         wL.eq(0),
                         wH.eq(1)]
        with m.Elif(motorstate == 4):  # W --> V, 110
            m.d.comb += [uL.eq(0),
                         uH.eq(0),
                         vL.eq(1),
                         vH.eq(0),
                         wL.eq(0),
                         wH.eq(1)]
        with m.Elif(motorstate == 5):  # U --> V, 100
            m.d.comb += [uL.eq(0),
                         uH.eq(1),
                         vL.eq(1),
                         vH.eq(0),
                         wL.eq(0),
                         wH.eq(0)]
        with m.Elif(motorstate == 6):  # U --> W, 101
            m.d.comb += [uL.eq(0),
                         uH.eq(1),
                         vL.eq(0),
                         vH.eq(0),
                         wL.eq(1),
                         wH.eq(0)]

        if mode == 'hallfilter':
            m.d.sync += self.debugword.eq(hallfilter)
        elif mode == 'cycletime':
            m.d.sync += self.debugword.eq(cycletime)
        elif mode == 'statecounter':
            m.d.sync += self.debugword.eq(statecounter)
        elif mode == 'anglecounter':
            m.d.sync += self.debugword.eq(countsperdegree)
        elif mode == 'angle':
            m.d.sync += self.debugword.eq(angle)
        elif mode == 'PIcontrol':
            m.d.sync += self.debugword.eq(Cat(countsperdegree,
                                              delay))
        else:
            raise Exception(f"{motorstate} not supported")

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
