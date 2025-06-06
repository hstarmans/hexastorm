from amaranth import Elaboratable, Module, Signal, signed
from amaranth.hdl import Array

from .config import MOVE_TICKS, bit_shift
from .resources import get_all_resources


class Polynomial(Elaboratable):
    """Polynomial motor controller

    Computes a polynomial of the form:
        pos(t) = c*t^3 + b*t^2 + a*t
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
        start       - Start signal
        ticklimit   - Number of ticks to evaluate the polynomial

    Outputs:
        step        - Step signal (Array)
        dir         - Direction signal (Array)
        busy        - Active when running
    """

    def __init__(self, platform=None, top=False):
        """
        platform  -- pass test platform
        top       -- trigger synthesis of module
        """
        self.top = top
        self.platform = platform
        self.divider = platform.clks[platform.hfosc_div]
        self.order = platform.poldegree
        self.bit_shift = bit_shift(platform)
        self.motors = platform.motors
        self.max_steps = int(MOVE_TICKS / 2)  # Nyquist

        # Input
        self.coeff = Array()
        for _ in range(self.motors):
            self.coeff.extend(
                [Signal(signed(self.bit_shift + 1)) for _ in range(self.order)]
            )
        self.start = Signal()
        self.ticklimit = Signal(MOVE_TICKS.bit_length())

        # Output
        self.busy = Signal()
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()
        # add 1 MHz clock domain
        cntr = Signal(range(self.divider))
        ticks = Signal(MOVE_TICKS.bit_length())

        # Internal signed counters per motor per order
        max_bits = (self.max_steps << self.bit_shift).bit_length()
        cntrs = Array(Signal(signed(max_bits + 1)) for _ in range(len(self.coeff)))
        prev = Array(Signal(signed(max_bits + 1)) for _ in range(self.motors))
        assert max_bits <= 64

        if self.top:
            steppers = get_all_resources(platform, "stepper")
            assert steppers, "No stepper resources found"
            for i, stepper in enumerate(steppers):
                m.d.comb += [
                    stepper.step.eq(self.step[i]),
                    stepper.dir.eq(self.dir[i]),
                ]
        else:
            self.ticks = ticks
            self.cntrs = cntrs

        # Step signal generation based on bit toggle
        for motor in range(self.motors):
            m.d.comb += self.step[motor].eq(cntrs[motor * self.order][self.bit_shift])

        # Direction detection
        for motor in range(self.motors):
            idx = motor * self.order
            m.d.sync += prev[motor].eq(cntrs[idx])
            with m.If(prev[motor] > cntrs[idx]):
                m.d.sync += self.dir[motor].eq(0)  # Negative
            with m.Elif(prev[motor] < cntrs[idx]):
                m.d.sync += self.dir[motor].eq(1)  # Positive

        with m.FSM(init="RESET", name="polynomen"):
            with m.State("RESET"):
                m.d.sync += self.busy.eq(0)
                m.next = "WAIT_START"

            with m.State("WAIT_START"):
                with m.If(self.start):
                    for motor in range(self.motors):
                        coef0 = motor * self.order
                        step_bit = self.bit_shift + 1
                        m.d.sync += [
                            cntrs[coef0].eq(cntrs[coef0][:step_bit]),
                            prev[motor].eq(prev[motor][:step_bit]),
                        ]
                        for degree in range(1, self.order):
                            m.d.sync += cntrs[coef0 + degree].eq(0)
                    m.d.sync += self.busy.eq(1)
                    m.next = "RUNNING"
                with m.Else():
                    m.d.sync += self.busy.eq(0)

            with m.State("RUNNING"):
                with m.If((ticks < self.ticklimit) & (cntr >= self.divider - 1)):
                    m.d.sync += [ticks.eq(ticks + 1), cntr.eq(0)]
                    for motor in range(self.motors):
                        idx = motor * self.order

                        op3, op2, op1 = 0, 0, 0
                        if self.order > 2:
                            op3 += 3 * 2 * self.coeff[idx + 2] + cntrs[idx + 2]
                            op2 += cntrs[idx + 2]
                            op1 += self.coeff[idx + 2] + cntrs[idx + 2]
                            m.d.sync += cntrs[idx + 2].eq(op3)
                        if self.order > 1:
                            op2 += 2 * self.coeff[idx + 1] + cntrs[idx + 1]
                            m.d.sync += cntrs[idx + 1].eq(op2)
                        op1 += (
                            self.coeff[idx + 1]
                            + self.coeff[idx]
                            + cntrs[idx + 1]
                            + cntrs[idx]
                        )
                        m.d.sync += cntrs[idx].eq(op1)
                with m.Elif(ticks < self.ticklimit):
                    m.d.sync += cntr.eq(cntr + 1)
                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = "WAIT_START"
        return m
