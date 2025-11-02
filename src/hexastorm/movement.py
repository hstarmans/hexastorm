from amaranth import Elaboratable, Module, Mux, Signal, signed
from amaranth.lib.io import Buffer
from amaranth.hdl import Array

from .resources import get_all_resources, StepperRecord


class Polynomial(Elaboratable):
    """Polynomial motor controller

        Evaluates:
            pos(t) = c·t³ + b·t² + a·t

        where t is a counter incremented every clock tick.

        The polynomial describes the stepper position of a single axis.
        A counter is used to capture the state of the polynomial.
        If a given bit, denoted by bitshift, of the counter changes,
        a step is sent to the motor.
        In every tick the step can at most increase
        with one count.
        Non step part of base Counters are kept after segment.
        A position is also kept internally.

        This code requires a lot of LUT, only order 2 is supported on UP5k
        It is assumed that the user can completely determine
        the outcome of the calculation.
        To ascertain step accuracy, c is submitted with a very high accuracy.
        For third order, this requires 41 bit wide numbers
        and is a "weakness" in the code.
        The code might be sped up via Horner's method and the use of DSPs.
        The current code does not require a DSP.

        Assumptions:
        max ticks per move is 10_000
        update frequency motor is 1 MHz

        Inputs:
        - `coeff`:      Array of coefficients [a, b, c] for each motor.
        - `start`:      Start signal to initiate polynomial evaluation.
        - `tick_limit`: Maximum number of ticks (duration) for the motion segment.
        - `step_laser`: External step signal override for the laser axis.
        - `dir_laser`:  External direction signal override for the laser axis.
        - `override_laser`: When high, overrides step/dir for the laser motor.

    Outputs:
        - `position`:   Current position of each motor in steps (signed).
        - `busy`:       High while polynomial evaluation is in progress.
    """

    def __init__(self, plf_cfg):
        """
        plf_cfg  -- platform configuration
        """
        self.plf_cfg = plf_cfg
        hdl_cfg = plf_cfg.hdl_cfg

        # Input
        self.coeff = Array(
            [
                Signal(signed(hdl_cfg.bit_shift + 1))
                for _ in range(hdl_cfg.motors * hdl_cfg.pol_degree)
            ]
        )
        self.start = Signal()
        self.tick_limit = Signal(hdl_cfg.move_ticks.bit_length())
        self.step_laser = Signal()
        self.dir_laser = Signal()
        self.override_laser = Signal()

        # Output
        self.busy = Signal()
        self.position = Array(Signal(signed(32)) for _ in range(hdl_cfg.motors))

        # Mixed
        self.steppers = [StepperRecord() for _ in range(hdl_cfg.motors)]

    def elaborate(self, platform):
        m = Module()
        steppers = self.steppers
        plf_cfg = self.plf_cfg
        hdl_cfg = plf_cfg.hdl_cfg
        ice40_cfg = plf_cfg.ice40_cfg

        # Timing counters
        divider = int(ice40_cfg["clks"][ice40_cfg["hfosc_div"]])
        cntr = Signal(range(divider))
        ticks = Signal(hdl_cfg.move_ticks.bit_length())

        # Internal signed counters per motor per order
        max_steps = int(hdl_cfg.move_ticks / 2)  # Nyquist
        max_bits = (max_steps << hdl_cfg.bit_shift).bit_length()
        cntrs = Array(Signal(signed(max_bits + 1)) for _ in range(len(self.coeff)))
        prev = Array(Signal(signed(max_bits + 1)) for _ in range(hdl_cfg.motors))
        assert max_bits <= 64

        if platform is None:
            # Expose internals for simulation
            self.ticks = ticks
            self.cntrs = cntrs
        else:
            # Connect to platform stepper resources
            steppers_res = get_all_resources(platform, "stepper", dir="-")
            for i, stepper in enumerate(steppers_res):
                m.submodules += [
                    step_buf := Buffer("o", stepper.step),
                    dir_buf := Buffer("o", stepper.dir),
                    lim_buf := Buffer("i", stepper.limit),
                ]
                m.d.comb += [
                    step_buf.o.eq(steppers[i].step),
                    dir_buf.o.eq(steppers[i].dir),
                    steppers[i].limit.eq(lim_buf.i),
                ]

        # Position
        # update position
        stepper_d = Array(Signal() for _ in range(hdl_cfg.motors))
        # assuming position is signed and 64 bits signals
        pos_max = pow(2, 32 - 1) - 2
        for idx, stepper in enumerate(self.steppers):
            pos = self.position[idx]
            m.d.sync += stepper_d[idx].eq(stepper.step)
            with m.If(stepper.limit == 1):
                m.d.sync += self.position[idx].eq(0)
            with m.Elif((pos > pos_max) | (pos < -pos_max)):
                m.d.sync += self.position[idx].eq(0)
            with m.Elif((stepper.step == 1) & (stepper_d[idx] == 0)):
                with m.If(stepper.dir):
                    m.d.sync += pos.eq(pos + 1)
                with m.Else():
                    m.d.sync += pos.eq(pos - 1)

        # Generate step pulse based on toggling specific bit
        laser_idx = list(plf_cfg.motor_cfg["steps_mm"].keys()).index(
            plf_cfg.motor_cfg["orth2lsrline"]
        )
        for motor in range(hdl_cfg.motors):
            idx = motor * hdl_cfg.pol_degree
            step_motor = cntrs[idx][hdl_cfg.bit_shift]

            with m.If(motor == laser_idx):
                # When you switch between sources, there should not be a pulse
                # 1. Create previous value registers
                step_motor_d = Signal()
                step_laser_d = Signal()

                m.d.sync += [
                    step_motor_d.eq(step_motor),
                    step_laser_d.eq(self.step_laser),
                ]

                # 2. Edge detect for each source
                edge_motor = Signal()
                edge_laser = Signal()

                m.d.sync += [
                    edge_motor.eq((step_motor == 1) & (step_motor_d == 0)),
                    edge_laser.eq((self.step_laser == 1) & (step_laser_d == 0)),
                ]

                # 3. Mux the *edges*, not the raw step signal
                m.d.sync += steppers[motor].step.eq(
                    Mux(self.override_laser, edge_laser, edge_motor)
                )
            with m.Else():
                m.d.sync += steppers[motor].step.eq(step_motor)

        # Direction signal based on delta between ticks
        for motor in range(hdl_cfg.motors):
            idx = motor * hdl_cfg.pol_degree
            m.d.sync += prev[motor].eq(cntrs[idx])

            with m.If((motor == laser_idx) & self.override_laser):
                # Override active for laser motor – use external dir
                m.d.sync += steppers[motor].dir.eq(self.dir_laser)
            with m.Else():
                with m.If(prev[motor] > cntrs[idx]):
                    m.d.sync += steppers[motor].dir.eq(0)  # Negative move
                with m.Elif(prev[motor] < cntrs[idx]):
                    m.d.sync += steppers[motor].dir.eq(1)  # Positive move

        # State machine for execution
        with m.FSM(init="RESET", name="polynomial_fsm"):
            with m.State("RESET"):
                m.d.sync += self.busy.eq(0)
                m.next = "WAIT_START"

            with m.State("WAIT_START"):
                with m.If(self.start):
                    for motor in range(hdl_cfg.motors):
                        coef0 = motor * hdl_cfg.pol_degree
                        step_bit = hdl_cfg.bit_shift + 1

                        for degree in range(1, hdl_cfg.pol_degree):
                            m.d.sync += cntrs[coef0 + degree].eq(0)

                        m.d.sync += [
                            cntrs[coef0].eq(cntrs[coef0][:step_bit]),
                            prev[motor].eq(prev[motor][:step_bit]),
                        ]

                    m.d.sync += self.busy.eq(1)
                    m.next = "RUNNING"
                with m.Else():
                    m.d.sync += self.busy.eq(0)

            with m.State("RUNNING"):
                # Tick handling and polynomial evaluation
                with m.If((ticks < self.tick_limit) & (cntr >= divider - 1)):
                    m.d.sync += [ticks.eq(ticks + 1), cntr.eq(0)]

                    for motor in range(hdl_cfg.motors):
                        base = motor * hdl_cfg.pol_degree
                        acc = [
                            Signal.like(cntrs[base + i])
                            for i in range(hdl_cfg.pol_degree)
                        ]

                        if hdl_cfg.pol_degree > 2:
                            acc[2] = 3 * 2 * self.coeff[base + 2] + cntrs[base + 2]
                            m.d.sync += cntrs[base + 2].eq(acc[2])
                            acc[1] = cntrs[base + 2]

                        if hdl_cfg.pol_degree > 1:
                            acc[1] = acc[1] + 2 * self.coeff[base + 1] + cntrs[base + 1]
                            m.d.sync += cntrs[base + 1].eq(acc[1])

                        acc[0] = (
                            self.coeff[base]
                            + self.coeff[base + 1]
                            + cntrs[base]
                            + cntrs[base + 1]
                        )
                        if hdl_cfg.pol_degree > 2:
                            acc[0] += self.coeff[base + 2] + cntrs[base + 2]
                        m.d.sync += cntrs[base].eq(acc[0])

                with m.Elif(ticks < self.tick_limit):
                    m.d.sync += cntr.eq(cntr + 1)

                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = "WAIT_START"
        return m
