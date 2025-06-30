from hexastorm.utils import LunaGatewareTestCase, async_test_case
from hexastorm.controller import TestHost
from hexastorm.config import PlatformConfig
from hexastorm.movement import Polynomial


class TestPolynomial(LunaGatewareTestCase):
    plf_cfg = PlatformConfig(test=True)
    FRAGMENT_UNDER_TEST = Polynomial
    FRAGMENT_ARGUMENTS = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim) -> None:
        """Initialize simulation environment and signals."""
        self.host = TestHost()
        self.move_ticks = self.plf_cfg.hdl_cfg.move_ticks
        self.sim = sim
        sim.set(self.dut.tick_limit, self.move_ticks)
        await sim.tick()

    async def count_steps(self, motor: int) -> int:
        """
        Count motor steps based on direction while DUT is busy.
        """
        sim = self.sim
        stepper = self.dut.steppers[motor]
        count = 0
        while sim.get(self.dut.busy):
            old = sim.get(stepper.step)
            await sim.tick()
            if old and sim.get(stepper.step) == 0:
                if sim.get(stepper.dir):
                    count += 1
                else:
                    count -= 1
        return count

    async def send_coefficients(self, a: int, b: int, c: int) -> None:
        """
        Send polynomial coefficients to DUT and pulse start.

        Coefficients are for: c·x³ + b·x² + a·x
        """
        coeffs = [a, b, c]
        hdl_cfg = self.plf_cfg.hdl_cfg

        for _ in range(hdl_cfg.motors):
            for idx in range(hdl_cfg.pol_degree):
                self.sim.set(self.dut.coeff[idx], coeffs[idx])
        await self.pulse(self.dut.start)

    @async_test_case
    async def test_tick_limit(self, sim):
        """Test motor step rate at various tick limits."""

        async def run_limit_test(limit: int, expected_steps: int):
            a = round(self.host.steps_to_count(expected_steps) / limit)
            sim.set(self.dut.tick_limit, limit)
            await sim.tick()
            await self.send_coefficients(a, 0, 0)
            actual_steps = await self.count_steps(0)
            self.assertEqual(actual_steps, expected_steps)

        await run_limit_test(self.move_ticks, 4000)
        await run_limit_test(10_000, 1)

    @async_test_case
    async def test_calculation(self, sim, a: int = 2, b: int = 3, c: int = 1) -> None:
        """Test basic polynomial evaluation: c·x³ + b·x² + a·x."""
        degr = self.plf_cfg.hdl_cfg.pol_degree

        if degr < 3:
            c = 0
        if degr < 2:
            b = 0

        await self.send_coefficients(a, b, c)

        while sim.get(self.dut.busy):
            await sim.tick()

        result = sim.get(self.dut.cntrs[0])
        x = self.move_ticks
        expected = a * x + b * x**2 + c * x**3
        self.assertEqual(result, expected)

    @async_test_case
    async def test_jerk(self, sim) -> None:
        """
        Test lowest non-zero coefficient for highest polynomial order
        results in exactly one step.
        """
        steps = 1
        degr = self.plf_cfg.hdl_cfg.pol_degree
        x = self.move_ticks
        coef = round(self.host.steps_to_count(steps) / (x**degr))

        coeffs = [0] * 3
        coeffs[degr - 1] = coef

        await self.send_coefficients(*coeffs)
        actual_steps = await self.count_steps(0)
        self.assertEqual(actual_steps, steps)

    @async_test_case
    async def test_move(self, sim) -> None:
        """Test forward and backward movement at constant speed."""

        async def run_move_test(steps: int):
            a = round(self.host.steps_to_count(steps) / self.move_ticks)
            await self.send_coefficients(a, 0, 0)
            actual = await self.count_steps(0)
            self.assertEqual(actual, steps)

        delta = round(0.4 * self.move_ticks)
        await run_move_test(delta)
        await run_move_test(-delta)
