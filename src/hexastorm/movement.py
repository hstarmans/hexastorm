from amaranth import Elaboratable, Module, Signal, signed
from amaranth.lib.io import Buffer
from amaranth.hdl import Array

from .resources import get_all_resources


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
    Higher orders, velocity etc are removed.

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
        coeff       - Array of polynomial coefficients [a, b, c] per motor
        start       - Start evaluation
        ticklimit   - Number of ticks to evaluate the polynomial

    Outputs:
        step        - Step signal per motor (Array)
        dir         - Direction signal per motor (Array)
        busy        - Active while polynomial is being evaluated
    """

    def __init__(self, plf_cfg):
        """
        plf_cfg  -- platform configuration
        """
        self.hdl_cfg = hdl_cfg = plf_cfg.hdl_cfg
        self.order = hdl_cfg.pol_degree
        self.motors = hdl_cfg.motors
        self.max_steps = int(hdl_cfg.move_ticks / 2)  # Nyquist
        self.bit_shift = hdl_cfg.bit_shift
        ice40_cfg = plf_cfg.ice40_cfg
        self.divider = ice40_cfg["clks"][ice40_cfg["hfosc_div"]]

        # Input
        self.coeff = Array()
        self.coeff = Array(
            [
                Signal(signed(self.bit_shift + 1))
                for _ in range(self.motors * self.order)
            ]
        )
        self.start = Signal()
        self.tick_limit = Signal(hdl_cfg.move_ticks.bit_length())

        # Output
        self.busy = Signal()
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()

        # Timing counters
        cntr = Signal(range(self.divider))
        ticks = Signal(self.hdl_cfg.move_ticks.bit_length())

        # Internal signed counters per motor per order
        max_bits = (self.max_steps << self.bit_shift).bit_length()
        cntrs = Array(Signal(signed(max_bits + 1)) for _ in range(len(self.coeff)))
        prev = Array(Signal(signed(max_bits + 1)) for _ in range(self.motors))
        assert max_bits <= 64

        if platform is None:
            # Expose internals for simulation
            self.ticks = ticks
            self.cntrs = cntrs
        else:
            # Connect to platform stepper resources
            steppers = get_all_resources(platform, "stepper", dir="-")
            assert steppers, "No stepper resources found"
            for i, stepper in enumerate(steppers):
                m.submodules += [
                    step_buf := Buffer("o", stepper.step),
                    dir_buf := Buffer("o", stepper.dir),
                ]
                m.d.comb += [
                    step_buf.o.eq(self.step[i]),
                    dir_buf.o.eq(self.dir[i]),
                ]

        # Generate step pulse based on toggling specific bit
        for motor in range(self.motors):
            idx = motor * self.order
            m.d.comb += self.step[motor].eq(cntrs[idx][self.bit_shift])

        # Direction signal based on delta between ticks
        for motor in range(self.motors):
            idx = motor * self.order
            m.d.sync += prev[motor].eq(cntrs[idx])
            with m.If(prev[motor] > cntrs[idx]):
                m.d.sync += self.dir[motor].eq(0)  # Negative move
            with m.Elif(prev[motor] < cntrs[idx]):
                m.d.sync += self.dir[motor].eq(1)  # Positive move

        # State machine for execution
        with m.FSM(init="RESET", name="polynomial_fsm"):
            with m.State("RESET"):
                m.d.sync += self.busy.eq(0)
                m.next = "WAIT_START"

            with m.State("WAIT_START"):
                with m.If(self.start):
                    for motor in range(self.motors):
                        coef0 = motor * self.order
                        step_bit = self.bit_shift + 1

                        for degree in range(1, self.order):
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
                with m.If((ticks < self.tick_limit) & (cntr >= self.divider - 1)):
                    m.d.sync += [ticks.eq(ticks + 1), cntr.eq(0)]

                    for motor in range(self.motors):
                        base = motor * self.order
                        acc = [Signal.like(cntrs[base + i]) for i in range(self.order)]

                        if self.order > 2:
                            acc[2] = 3 * 2 * self.coeff[base + 2] + cntrs[base + 2]
                            m.d.sync += cntrs[base + 2].eq(acc[2])
                            acc[1] = cntrs[base + 2]

                        if self.order > 1:
                            acc[1] = acc[1] + 2 * self.coeff[base + 1] + cntrs[base + 1]
                            m.d.sync += cntrs[base + 1].eq(acc[1])

                        acc[0] = (
                            self.coeff[base]
                            + self.coeff[base + 1]
                            + cntrs[base]
                            + cntrs[base + 1]
                        )
                        if self.order > 2:
                            acc[0] += self.coeff[base + 2] + cntrs[base + 2]
                        m.d.sync += cntrs[base].eq(acc[0])

                with m.Elif(ticks < self.tick_limit):
                    m.d.sync += cntr.eq(cntr + 1)

                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = "WAIT_START"
        return m
