import unittest

import numpy as np
from amaranth import Cat, Elaboratable, Module, Signal
from amaranth.hdl.mem import Array
from luna.gateware.interface.spi import SPICommandInterface

from .constants import WORD_BYTES, params
from .resources import get_all_resources


class Driver(Elaboratable):
    """Drives 3 pole BLDC motor

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
    O: bldc (6)       -- power bridges BLDC motor
    O: debugmotor     -- word sent over via SPI to debug motor
    I: synchronized   -- motor synchronized with laser
    I: ticksinfacet   -- number of ticks in facet
    """

    def __init__(self, platform, top=False, divider=800, PIcontrol=True):
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
        self.debugword = Signal(WORD_BYTES * 8)
        # optical encoder
        self.synchronized = Signal()
        self.dct = params(platform)
        self.ticksinfacet = Signal(range(self.dct["TICKSINFACET"] * 2))

        # depending on mode, a certain word is sent back
        self.mode = platform.laser_var["MOTORDEBUG"]
        self.divider = divider
        self.PIcontrol = PIcontrol

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(range(8))
        hallstate = Signal().like(motorstate)
        hallfilter = Signal().like(hallstate)
        # total number of states in 360 degrees
        # Record how a rotation takes place; e.g. red (1), cyan (5), 
        # darkblue (4), lightblue (6), green (2), yellow (3)
        # map this to the motor state; 1-->1, 5-->2, 4-->3, 6-->4, 
        # 2-->5, 3-->6, see HALL mode in the statemachine
        # optionally rotate the mapping for alignment
        # yellow, i.e. state 3 does not propagate
        # due to hardware error
        # so not 12 but 10 in total
        states_fullcycle = 10
        angle = Signal(range(180))
        # angle counter limits
        mode = self.mode

        uL, uH, vL, vH, wL, wH = (
            self.uL,
            self.uH,
            self.vL,
            self.vH,
            self.wL,
            self.wH,
        )

        if platform and self.top:
            leds = [res.o for res in get_all_resources(platform, "led")]
            for idx in range(len(self.leds)):
                m.d.comb += self.leds[idx].eq(leds[idx])
            bldc = platform.request("bldc")
            m.d.comb += self.enable_prism.eq(1)
            m.d.comb += [
                bldc.uL.eq(uL),
                bldc.uH.eq(uH),
                bldc.vL.eq(vL),
                bldc.vH.eq(vH),
                bldc.wL.eq(wL),
                bldc.wH.eq(wH),
            ]
            m.d.comb += [
                hallstate.eq(Cat(~bldc.sensor0, ~bldc.sensor1, ~bldc.sensor2))
            ]

            spi = self.spi
            interf = SPICommandInterface(command_size=1 * 8, word_size=4 * 8)
            m.d.comb += interf.spi.connect(spi)
            m.submodules.interf = interf

        # displays measurement hall sesnsors on LED
        for idx in range(len(self.leds)):
            m.d.comb += self.leds[idx].eq(self.hall[idx])

        # tested by trying out all possibilities
        m.d.sync += hallstate.eq(
            Cat(~self.hall[0], ~self.hall[1], ~self.hall[2])
        )

        def get_statetime(frequency):
            """time needed for one state"""
            return int(
                (platform.laser_var["CRYSTAL_HZ"])
                / (frequency * states_fullcycle)
            )

        start_freq = 2  # Hz

        # angle counter limits
        #    it is calculated how much time 1 degree takes
        #    it assumed speed must be between 100 and 10 hertz
        lowerlimit = int((get_statetime(100)*states_fullcycle/360))
        upperlimit = int((get_statetime(start_freq)*states_fullcycle/360))

        # clock is downscaled to filter out
        # noise coming to the hall sensors
        # this creates hall filter
        divider_cnt = Signal(range(self.divider // 2))
        with m.If(divider_cnt == self.divider // 2 - 1):
            m.d.sync += [divider_cnt.eq(0), hallfilter.eq(hallstate)]
        with m.Else():
            m.d.sync += divider_cnt.eq(divider_cnt + 1)


        # counts per degree enables you
        # to calculate the motor angle
        # two bytes are used so we can read it out via SPI
        # assumption 16 bit >> (max_halfrotationtime/360)
        countsperdegree = Signal(16)
        countsperdegreediode = Signal.like(countsperdegree)

        # counts per degree of the hall sensors should equal
        # equal the facet time retrieved by laser
        # TODO: number do not allign 
        ticksinfacet = self.ticksinfacet
        m.d.sync += countsperdegreediode.eq(
                        (ticksinfacet >> 7)
                        + (ticksinfacet >> 9)
                        + (ticksinfacet >> 10)
                        + (ticksinfacet >> 12)
                        + (ticksinfacet >> 13)
                    )

        # counter used in state ROTATION
        start_statetime = get_statetime(start_freq)
        mtrpulsecntr = Signal(range(int((start_statetime + 1))))

        # if the motor is rotating we switch from
        # rotation to hall and vice versa
        rotating = Signal()

        with m.FSM(reset="ROTATION", name="algo"):
            with m.State("ROTATION"):
                with m.If(mtrpulsecntr == start_statetime):
                    m.d.sync += mtrpulsecntr.eq(0)
                    with m.If(motorstate == 6):
                        m.d.sync += motorstate.eq(1)
                    with m.Else():
                        m.d.sync += motorstate.eq(motorstate + 1)
                with m.Else():
                    m.d.sync += mtrpulsecntr.eq(mtrpulsecntr + 1)
                with m.If(rotating == 1):
                    m.next = "HALL"
        # Hall feedback mode is not optimal as the measurements
        # by the sensors are not equidistant, probably due to
        # sensor limitation
        # PID controller does not work with this mode
            with m.State("HALL"):
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
                    m.next = "ROTATION"
                with m.Elif(
                    (countsperdegree > lowerlimit)
                    & (countsperdegree < upperlimit)
                    & (mode != "hallfilter")
                ):
                    m.next = "IMPHALL"
            with m.State("IMPHALL"):
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
                    m.next = "ROTATION"


        max_rotationtime = int(start_statetime * states_fullcycle)
        hallcntr = Signal.like(range(max_rotationtime))
        # degree counter
        degree_cnt = Signal().like(countsperdegree)

        # invalid hall measurements are filtered out
        # in addition focus is on changes
        statecounter = Signal(range(states_fullcycle // 2))
        stateold = Signal.like(hallstate)
        startangle = 0

        # counts per degree measurement
        # note that we require half a rotation for a update
        with m.If(
            (hallfilter != stateold) & (hallfilter > 0) & (hallfilter != 3) &
             (hallfilter < 7)
        ):
            m.d.sync += stateold.eq(hallfilter)
            with m.If(statecounter == states_fullcycle // 2 - 1):
                # TODO: add filter to remove invalid cycle times,
                #       which are too small
                m.d.sync += [
                    statecounter.eq(0),
                    hallcntr.eq(0),
                    # division by 180 via bitshifts
                    countsperdegree.eq(
                        (hallcntr >> 8)
                        + (hallcntr >> 10)
                        + (hallcntr >> 11)
                        + (hallcntr >> 13)
                        + (hallcntr >> 14)
                    ),
                    degree_cnt.eq(0),
                    angle.eq(startangle),
                    # rotating so use hall sensors
                    rotating.eq(1)]
            with m.Else():
                m.d.sync += statecounter.eq(statecounter + 1)
        # counter is overflowing, implying there is no rotation
        with m.Elif(hallcntr == int(start_statetime * states_fullcycle)):
            m.d.sync += [hallcntr.eq(0), rotating.eq(0)]
        with m.Else():
            m.d.sync += hallcntr.eq(hallcntr + 1)
            # ideally we would also rotate counter in other blocks
            # of if else ladder but error assumed minimal
            with m.If((degree_cnt + 1) > countsperdegree):
                m.d.sync += degree_cnt.eq(0)
                with m.If(angle == 180-1):
                    m.d.sync += angle.eq(0)
                with m.Else():
                    m.d.sync += angle.eq(angle + 1)
            with m.Else():
                m.d.sync += degree_cnt.eq(degree_cnt + 1)

        # PID controller
        #
        # Target is rotation speed. Power motor is reduced
        # by temporarily placing it in on mode during the
        # duty time. The cycle time is the PID cycle time.
        # A new measurement is obtained after 180 degrees
        # It then traverses 6 states, one state is 30 degrees.
        # Duty time is typically very small as compared to
        # PID cycle time. We want multiple
        # cycles per 180 degrees to ensure it is position 
        # independent.

        # target speed
        clock = int(platform.clks[platform.hfosc_div] * 1e6)
        RPM = platform.laser_var["RPM"]
        setpoint_degree = int(round((clock / (2 * RPM / 60))) / 180)

        # desire 50 samples per cycle
        max_pid_cycle_time = int(180 / 50 * setpoint_degree)

        duty = Signal(range(max_pid_cycle_time))

        # limits are caclulated to prevent overflow, i.e. integral blow up
        # TODO: seem excessive and suboptimal
        assert (upperlimit > lowerlimit) & (setpoint_degree > 0) & (lowerlimit > 0)
        lower_l = setpoint_degree - upperlimit
        upper_l = setpoint_degree - lowerlimit
        # TODO: this is excessive
        int_lower_l = lower_l * 1000
        int_upper_l = upper_l * 1000
        err = Signal(range(lower_l, upper_l))
        assert (upper_l > 0) & (lower_l < 0)
        # der = Signal(range(lower_l - upper_l, upper_l - lower_l))
        intg = Signal(range(int_lower_l, int_upper_l))

        with m.If(rotating == 0):
            m.d.sync += [duty.eq(max_pid_cycle_time-1), err.eq(0), intg.eq(0)]
        with m.Else():
            # ugly use state machine!
            with m.If(hallcntr == 0):
                m.d.sync += [
                    err.eq(setpoint_degree - countsperdegree),
                    # der.eq(err - setpoint + countsperdegree),
                    intg.eq(intg + err),
                ]
            with m.Elif(hallcntr  == 1):
                # normally you multiply by constants but 
                # we use bit shift to avoid multiplications
                m.d.sync += duty.eq(-(err >> 3) - (intg >> 10))

        off = Signal()
        pid_cycle_time = Signal(range(max_pid_cycle_time))

        # Computes position in pid_cycles
        with m.If(pid_cycle_time < max_pid_cycle_time):
            m.d.sync += pid_cycle_time.eq(0)
        with m.Else():
            m.d.sync += pid_cycle_time.eq(pid_cycle_time + 1)

        # Motor On
        with m.If(pid_cycle_time < duty):
            m.d.sync += off.eq(0)
        # Motor off
        with m.Else():
            m.d.sync += off.eq(1)

        # the are three connections U, V, W
        # order is not trivial see literature
        # https://www.mathworks.com/help/mcb/ref/sixstepcommutation.html
        with m.If(((off & self.PIcontrol) & rotating) | (~self.enable_prism)):
            m.d.comb += [
                uL.eq(0),
                uH.eq(0),
                vL.eq(0),
                vH.eq(0),
                wL.eq(0),
                wH.eq(0),
            ]
        with m.Elif(motorstate == 2):  # V --> W
            m.d.comb += [
                uL.eq(0),
                uH.eq(0),
                vL.eq(0),
                vH.eq(1),
                wL.eq(1),
                wH.eq(0),
            ]
        with m.Elif(motorstate == 3):  # V --> U
            m.d.comb += [
                uL.eq(1),
                uH.eq(0),
                vL.eq(0),
                vH.eq(1),
                wL.eq(0),
                wH.eq(0),
            ]
        with m.Elif(motorstate == 4):  # W --> U
            m.d.comb += [
                uL.eq(1),
                uH.eq(0),
                vL.eq(0),
                vH.eq(0),
                wL.eq(0),
                wH.eq(1),
            ]
        with m.Elif(motorstate == 5):  # W --> V
            m.d.comb += [
                uL.eq(0),
                uH.eq(0),
                vL.eq(1),
                vH.eq(0),
                wL.eq(0),
                wH.eq(1),
            ]
        with m.Elif(motorstate == 6):  # U --> V
            m.d.comb += [
                uL.eq(0),
                uH.eq(1),
                vL.eq(1),
                vH.eq(0),
                wL.eq(0),
                wH.eq(0),
            ]
        with m.Elif(motorstate == 1):  # U --> W
            m.d.comb += [
                uL.eq(0),
                uH.eq(1),
                vL.eq(0),
                vH.eq(0),
                wL.eq(1),
                wH.eq(0),
            ]

        if mode == "hallfilter":
            m.d.sync += self.debugword.eq(hallfilter)
        elif mode == "statecounter":
            m.d.sync += self.debugword.eq(statecounter)
        elif mode == "ticksinfacet":
            m.d.sync += self.debugword.eq(Cat(countsperdegree, countsperdegreediode))
        elif mode == "angle":
            m.d.sync += self.debugword.eq(angle)
        elif mode == "PIcontrol":
            m.d.sync += self.debugword.eq(Cat(countsperdegree, duty))
        else:
            raise Exception(f"{motorstate} not supported")

        return m


if __name__ == "__main__":
    unittest.main()
