import unittest

from amaranth import Elaboratable, Module, Signal, signed
from amaranth.hdl import Array

from .utils import LunaGatewareTestCase, async_test_case
from .constants import MOVE_TICKS, bit_shift
from .controller import Host
from .platforms import TestPlatform
from .resources import get_all_resources


class Polynomal(Elaboratable):
    """Sets motor states using a polynomal algorithm

    A polynomal up to 3 order, i.e. c*t^3+b*t^2+a*t,
    is evaluated under the assumption that t starts at 0
    and has a maximum of say 10_000 ticks.
    The polynomal describes the stepper position of a single axis.
    A counter is used to capture the state of the polynomal.
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

    I/O signals:
    I: coeff          -- polynomal coefficients
    I: start          -- start signal
    O: busy           -- busy signal
    O: finished       -- finished signal
    O: total steps    -- total steps executed in move
    O: dir            -- direction; 1 is postive and 0 is negative
    O: step           -- step signal
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
        # inputs
        self.coeff = Array()
        for _ in range(self.motors):
            self.coeff.extend(
                [
                    Signal(signed(self.bit_shift + 1)),
                    Signal(signed(self.bit_shift + 1)),
                    Signal(signed(self.bit_shift + 1)),
                ][: self.order]
            )
        self.start = Signal()
        self.ticklimit = Signal(MOVE_TICKS.bit_length())
        # output
        self.busy = Signal()
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()
        # add 1 MHz clock domain
        cntr = Signal(range(self.divider))
        # pos
        max_bits = (self.max_steps << self.bit_shift).bit_length()
        cntrs = Array(
            Signal(signed(max_bits + 1)) for _ in range(len(self.coeff))
        )
        assert max_bits <= 64
        ticks = Signal(MOVE_TICKS.bit_length())
        if self.top:
            steppers = [res for res in get_all_resources(platform, "stepper")]
            assert len(steppers) != 0
            for idx, stepper in enumerate(steppers):
                m.d.comb += [
                    stepper.step.eq(self.step[idx]),
                    stepper.dir.eq(self.dir[idx]),
                ]
        else:
            self.ticks = ticks
            self.cntrs = cntrs

        # steps
        for motor in range(self.motors):
            m.d.comb += self.step[motor].eq(
                cntrs[motor * self.order][self.bit_shift]
            )

        # directions
        counter_d = Array(
            Signal(signed(max_bits + 1)) for _ in range(self.motors)
        )
        for motor in range(self.motors):
            m.d.sync += counter_d[motor].eq(cntrs[motor * self.order])
            # negative case --> decreasing
            with m.If(counter_d[motor] > cntrs[motor * self.order]):
                m.d.sync += self.dir[motor].eq(0)
            # positive case --> increasing
            with m.Elif(counter_d[motor] < cntrs[motor * self.order]):
                m.d.sync += self.dir[motor].eq(1)
        with m.FSM(init="RESET", name="polynomen"):
            with m.State("RESET"):
                m.next = "WAIT_START"

                m.d.sync += self.busy.eq(0)
            with m.State("WAIT_START"):
                with m.If(self.start):
                    for motor in range(self.motors):
                        coef0 = motor * self.order
                        step_bit = self.bit_shift + 1
                        m.d.sync += [
                            cntrs[coef0].eq(cntrs[coef0][:step_bit]),
                            counter_d[motor].eq(counter_d[motor][:step_bit]),
                        ]
                        for degree in range(1, self.order):
                            m.d.sync += cntrs[coef0 + degree].eq(0)
                    m.d.sync += self.busy.eq(1)
                    m.next = "RUNNING"
                with m.Else():
                    m.d.sync += self.busy.eq(0)
            with m.State("RUNNING"):
                with m.If(
                    (ticks < self.ticklimit) & (cntr >= self.divider - 1)
                ):
                    m.d.sync += [ticks.eq(ticks + 1), cntr.eq(0)]
                    for motor in range(self.motors):
                        order = self.order
                        idx = motor * order
                        op3, op2, op1 = 0, 0, 0
                        if order > 2:
                            op3 += 3 * 2 * self.coeff[idx + 2] + cntrs[idx + 2]
                            op2 += cntrs[idx + 2]
                            op1 += self.coeff[idx + 2] + cntrs[idx + 2]
                            m.d.sync += cntrs[idx + 2].eq(op3)
                        if order > 1:
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


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {"platform": platform}

    async def initialize_signals(self, sim):
        self.host = Host(self.platform)
        sim.set(self.dut.ticklimit, MOVE_TICKS)
        await sim.tick()

    async def count_steps(self, sim, motor):
        """counts steps in accounting for direction"""
        count = 0
        while sim.get(self.dut.busy):
            old = sim.get(self.dut.step[motor])
            await sim.tick()
            if old and sim.get(self.dut.step[motor]) == 0:
                if sim.get(self.dut.dir[motor]):
                    count += 1
                else:
                    count -= 1
        return count

    async def send_coefficients(self, sim, a, b, c):
        """send coefficients and pulse start

        a,b,c --  for cx^3+bx^2+ax
        """
        coefs = [a, b, c]
        for _ in range(self.platform.motors):
            for coef in range(self.dut.order):
                sim.set(self.dut.coeff[coef], coefs[coef])
        await self.pulse(sim, self.dut.start)

    @async_test_case
    async def test_ticklimit(self, sim):
        """Test max speed limit behavior."""

        async def limittest(limit, steps):
            a = round(self.host.steps_to_count(steps) / limit)
            sim.set(self.dut.ticklimit, limit)
            await sim.tick()
            await self.send_coefficients(sim, a, 0, 0)
            step_count = await self.count_steps(sim, 0)
            self.assertEqual(step_count, steps)

        await limittest(MOVE_TICKS, 4000)
        await limittest(10_000, 1)

    @async_test_case
    async def test_calculation(self, sim, a=2, b=3, c=1):
        """Test a simple relation cx^3 + bx^2 + ax"""
        if self.dut.order < 3:
            c = 0
        if self.dut.order < 2:
            b = 0

        await self.send_coefficients(sim, a, b, c)

        while sim.get(self.dut.busy):
            await sim.tick()

        result = sim.get(self.dut.cntrs[0])
        expected = a * MOVE_TICKS + b * MOVE_TICKS**2 + c * MOVE_TICKS**3
        self.assertEqual(result, expected)

    @async_test_case
    async def test_jerk(self, sim):
        """Test minimum coefficient for highest-order term"""
        steps = 1
        coef = round(self.host.steps_to_count(steps) / (MOVE_TICKS**self.dut.order))
        coeffs = [0] * 3
        coeffs[self.dut.order - 1] = coef
        await self.send_coefficients(sim, *coeffs)
        step_count = await self.count_steps(sim, 0)
        self.assertEqual(step_count, steps)

    @async_test_case
    async def test_move(self, sim):
        """Forward and backward move at constant speed."""

        async def do_move(steps):
            a = round(self.host.steps_to_count(steps) / MOVE_TICKS)
            await self.send_coefficients(sim, a, 0, 0)
            count = await self.count_steps(sim, 0)
            self.assertEqual(count, steps)

        steps = round(0.4 * MOVE_TICKS)
        await do_move(steps)
        await do_move(-steps)


if __name__ == "__main__":
    unittest.main()
