import math

from amaranth import Cat, Elaboratable, Module, Signal, ClockDomain
from amaranth.hdl import Array
from luna.gateware.interface.spi import SPICommandInterface
from luna.gateware.utils.cdc import synchronize

from .constants import WORD_BYTES, COMMAND_BYTES, params
from .resources import get_all_resources


class Driver(Elaboratable):
    """Drives three poles BLDC motor

    Motor rotates clockwise.
    Motor driven via six step commutation cycle.
    Driver starts rotation without using hall sensors and
    simply steps through the motor drive loop.
    Once rotating, it starts to rely on the hall sensors
    and speeds up.
    The speed is calculated via the Hall sensors.
    A positional integral derivative (PID) controller
    in combination with PWM (Pulse Width Modulation)
    is used to stabilize the rotor speed.

    I: enable_prism   -- enable pin scanner motor
    I: Hall (3)       -- hall sensor measure state motor
    O: leds (3)       -- display measurements Hall sensors
    O: bldc (6)       -- power bridges BLDC motor
    O: debugmotor     -- word sent over via SPI to debug motor
    I: synchronized   -- motor synchronized with laser
    I: ticksinfacet   -- number of ticks in facet
    """

    def __init__(self, platform, top=False, speedfix=True):
        """
        platform  -- pass test platform
        top       -- True if top module
        speedfix  -- speed is stabilized
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
        self.divider = platform.laser_var["MOTORDIVIDER"]
        self.spinuptimme = platform.laser_var["SPINUP_TIME"]
        self.PIcontrol = speedfix

    def elaborate(self, platform):
        m = Module()
        motorstate = Signal(range(8))
        hallstate = Signal().like(motorstate)

        # total number of states in 360 degrees
        #  8 magnets, 3 sensors --> 24 states
        # Record how a rotation takes place; e.g. red (1), cyan (5),
        # darkblue (4), lightblue (6), green (2), yellow (3)
        # map this to the motor state; 1 --> 1, 5 --> 2, 4 --> 3, 6 --> 4,
        # 2 --> 5, 3 --> 6, see HALL mode in the statemachine
        # optionally rotate the mapping for alignment
        # yellow, i.e. state 3 does not propagate
        valid_states = [1, 2, 4, 5, 6]
        states_fullcycle = len(valid_states) * 4
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
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
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
                self.hall[0].eq(bldc.sensor0),
                self.hall[1].eq(bldc.sensor1),
                self.hall[2].eq(bldc.sensor2),
            ]

            interf = SPICommandInterface(
                command_size=COMMAND_BYTES * 8, word_size=WORD_BYTES * 8
            )
            m.d.comb += [
                interf.spi.connect(spi),
                interf.word_to_send.eq(self.debugword),
            ]
            m.submodules.interf = interf
        else:
            platform = self.platform

        # color LED displays hall state
        for idx in range(len(self.leds)):
            m.d.comb += self.leds[idx].eq(self.hall[idx])

        m.d.comb += hallstate.eq(
            Cat(~self.hall[0], ~self.hall[1], ~self.hall[2])
        )

        def get_statetime(frequency):
            """time needed for one state"""
            return int(
                (platform.laser_var["CRYSTAL_HZ"])
                / (frequency * states_fullcycle * self.divider)
            )

        start_freq = 2  # Hz

        # clock is downscaled to filter out
        # noise coming to hall sensors
        slow = ClockDomain("slow")
        m.domains += slow

        divider_cnt = Signal(range(self.divider // 2))
        with m.If(divider_cnt == self.divider // 2 - 1):
            m.d.sync += [divider_cnt.eq(0), slow.clk.eq(~slow.clk)]
        with m.Else():
            m.d.sync += divider_cnt.eq(divider_cnt + 1)

        # counter used in state ROTATION
        start_statetime = get_statetime(start_freq)
        mtrpulsecntr = Signal(range(int((start_statetime + 1))))

        # if the motor is rotating we switch from
        # rotation to hall and vice versa
        rotating = Signal()

        with m.FSM(init="ROTATION", name="algo"):
            with m.State("ROTATION"):
                with m.If(mtrpulsecntr == start_statetime):
                    m.d.slow += mtrpulsecntr.eq(0)
                    with m.If(motorstate == 1):
                        m.d.slow += motorstate.eq(6)
                    with m.Else():
                        m.d.slow += motorstate.eq(motorstate - 1)
                with m.Else():
                    m.d.slow += mtrpulsecntr.eq(mtrpulsecntr + 1)
                with m.If(rotating == 1):
                    m.next = "HALL"
            # Hall feedback mode is not optimal as the measurements
            # by the sensors are not equidistant, probably due to
            # sensor limitation
            with m.State("HALL"):
                with m.If(hallstate == 1):
                    m.d.slow += motorstate.eq(1)
                with m.Elif(hallstate == 2):
                    m.d.slow += motorstate.eq(3)
                with m.Elif(hallstate == 3):
                    m.d.slow += motorstate.eq(2)
                with m.Elif(hallstate == 4):
                    m.d.slow += motorstate.eq(5)
                with m.Elif(hallstate == 5):
                    m.d.slow += motorstate.eq(6)
                with m.Elif(hallstate == 6):
                    m.d.slow += motorstate.eq(4)
                # hall off and all hall on are filtered
                # i.e. cases 0 en 7
                with m.If(rotating == 0):
                    m.next = "ROTATION"

        hallcntr = Signal(range(start_statetime))
        hall_counters = Array(Signal.like(hallcntr) for _ in range(6))
        # must be multiple of 8 to read out
        # TODO: fix
        assert int(start_statetime * (states_fullcycle / 2)) < pow(2, 16)
        ticks_facet_hall = Signal(16)
        ticks_facet_diode = Signal.like(ticks_facet_hall)
        diode_shift = int(math.log(self.divider) / math.log(2))

        # CHALLENGE:
        #  you expect that each state takes 1/6 of time
        #  this is not true in practice, in addition we don't measure
        #  state 3.. to align hall and diode I use a constant
        clock = int(self.platform.clks[self.platform.hfosc_div] * 1e6)
        #  denk 0.12  vewacht 1/6 --> 0.16, 0.06  1982
        offset = int(
            0.065
            * clock
            / (
                self.platform.laser_var["RPM"]
                * 4
                * self.platform.laser_var["MOTORDIVIDER"]
            )
            * 60
        )
        m.d.sync += [
            ticks_facet_diode.eq(self.ticksinfacet >> diode_shift),
            ticks_facet_hall.eq(sum(hall_counters) + offset),
        ]

        # a functional PI controller requires speed changes to
        # propagate as fast as possible.
        # Speed is updated after each measuremement.
        # Hall states are not equally distributed some take longer than others
        # to mitigate this I look at the sum of all hall states.

        stateold = Signal.like(hallstate)
        with m.If(
            (hallstate != stateold)
            & (hallstate > 0)
            & (hallstate != 3)
            & (hallstate < 7)
        ):
            m.d.slow += [hallcntr.eq(0), stateold.eq(hallstate)]
            with m.If(hallstate == 1):
                m.d.slow += hall_counters[0].eq(hallcntr)
            with m.Elif(hallstate == 2):
                m.d.slow += hall_counters[1].eq(hallcntr)
            with m.Elif(hallstate == 3):
                m.d.slow += hall_counters[2].eq(hallcntr)
            with m.Elif(hallstate == 4):
                m.d.slow += hall_counters[3].eq(hallcntr)
            with m.Elif(hallstate == 5):
                m.d.slow += hall_counters[4].eq(hallcntr)
            with m.Elif(hallstate == 6):
                m.d.slow += hall_counters[5].eq(hallcntr)
            with m.If(
                ticks_facet_hall
                < int(start_statetime * (states_fullcycle / 2))
            ):
                m.d.slow += rotating.eq(1)
        # counter is overflowing, implying there is no rotation
        with m.Elif(hallcntr == start_statetime - 1):
            m.d.slow += [hallcntr.eq(0), rotating.eq(0)]
        with m.Else():
            m.d.slow += hallcntr.eq(hallcntr + 1)

        # Speed control via PID and PWM.
        #
        # Target is rotation speed. Power motor is reduced
        # by temporarily placing it in on mode during the
        # duty time, i.e. via pulse width modulation (PWM).
        # The duty time is determined via PID controller.
        # Tuning a controller is hard, try a wide range.

        # target speed
        clock = int(platform.clks[platform.hfosc_div] * 1e6)
        RPM = platform.laser_var["RPM"]
        # ticks in facet --> divide by 4
        setpoint_ticks = int(round((clock / (self.divider * 4 * RPM / 60))))

        # desire 10 samples per half rotation
        max_pid_cycle_time = (setpoint_ticks * self.divider) // 6
        duty = Signal(range(max_pid_cycle_time))

        # limits are caclulated to prevent overflow, i.e. integral blow up

        # half rotation counter limits
        #    it assumed speed must be between 100 and 10 hertz
        lowerlimit = get_statetime(100) * states_fullcycle
        upperlimit = get_statetime(start_freq) * states_fullcycle

        if not self.platform.name == "Test":
            assert (
                (upperlimit > lowerlimit)
                & (setpoint_ticks > 0)
                & (lowerlimit > 0)
            )
        lower_l = int(-setpoint_ticks)
        upper_l = int(upperlimit - setpoint_ticks)
        assert lower_l < 0
        err = Signal(range(lower_l, upper_l))
        duty = Signal(range(lower_l, upper_l))

        # integration time, assumed less than 10 seconds
        integration_ticks = (RPM // 60) * states_fullcycle * self.spinuptimme
        int_lower_l = int(lower_l * integration_ticks)
        int_upper_l = int(upper_l * integration_ticks)
        intg = Signal(range(int_lower_l, int_upper_l))

        K_p = 6  # proportionallity constant
        K_i = 15  # integration constant

        with m.If(rotating == 0):
            m.d.slow += [
                duty.eq(max_pid_cycle_time - 1),
                err.eq(0),
                intg.eq(0),
            ]
        with m.Elif(hallcntr == 0):
            # with m.If(self.synchronized):
            #     m.d.slow += [
            #         err.eq(ticks_half_rotation_diode - setpoint_ticks),
            #     ]
            # with m.Else():
            m.d.slow += [
                err.eq(ticks_facet_hall - setpoint_ticks),
            ]
        with m.Elif(hallcntr == 1):
            with m.If(
                ((intg + err) > int_lower_l) & ((intg + err) < int_upper_l)
            ):
                m.d.slow += intg.eq(intg + err)
        with m.Else():
            m.d.slow += [duty.eq((err >> K_p) + (intg >> K_i))]

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

        if mode == "hallstate":
            m.d.sync += self.debugword.eq(hallstate)
        elif mode == "ticksinfacet":
            m.d.sync += self.debugword.eq(
                Cat(ticks_facet_hall, ticks_facet_diode)
            )
        elif mode == "PIcontrol":
            m.d.sync += self.debugword.eq(Cat(ticks_facet_hall, duty))
        else:
            raise Exception(f"{motorstate} not supported")

        return m

