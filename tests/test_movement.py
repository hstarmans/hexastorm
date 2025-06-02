from hexastorm.utils import LunaGatewareTestCase, async_test_case
from hexastorm.controller import Host
from hexastorm.platforms import TestPlatform
from hexastorm.constants import MOVE_TICKS
from hexastorm.movement import Polynomial

class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomial
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