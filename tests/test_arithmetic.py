import random

from hexastorm.arithmetic import Multiplication, Divisor
from hexastorm.utils import LunaGatewareTestCase, async_test_case


class MultiplicationTest(LunaGatewareTestCase):
    """Unit test for the Multiplication hardware module."""

    FRAGMENT_UNDER_TEST = Multiplication

    @async_test_case
    async def test_multiply(self, sim):
        """Test a fixed multiplication result."""
        a, b = 319, 40
        expected = a * b

        sim.set(self.dut.a, a)
        sim.set(self.dut.b, b)

        await sim.tick()
        await sim.tick()  # Allow for propagation/delay if needed

        result = sim.get(self.dut.c)
        self.assertEqual(result, expected, f"{a} * {b} != {result}")


class DivisorTest(LunaGatewareTestCase):
    """Unit tests for the Divisor hardware module."""

    FRAGMENT_UNDER_TEST = Divisor
    FRAGMENT_ARGUMENTS = {"width": 6}

    async def initialize_signals(self, sim):
        self.sim = sim

    async def divide(self, x, y):
        """Run a division test on the DUT."""
        sim = self.sim
        width = self.FRAGMENT_ARGUMENTS["width"]

        assert x.bit_length() <= width, f"x={x} exceeds bit width"
        assert y.bit_length() <= width, f"y={y} exceeds bit width"

        sim.set(self.dut.x, x)
        sim.set(self.dut.y, y)
        sim.set(self.dut.start, 1)
        await sim.tick()
        sim.set(self.dut.start, 0)
        await sim.tick()

        # Wait for valid output or devide-by-zero flag
        for _ in range(100):  # Add a timeout to avoid infinite loop
            if sim.get(self.dut.valid) or sim.get(self.dut.dbz):
                break
            await sim.tick()
        else:
            self.fail(f"Division timed out: x={x}, y={y}")

        if sim.get(self.dut.valid):
            self.assertEqual(sim.get(self.dut.q), x // y)
            self.assertEqual(sim.get(self.dut.r), x % y)
        else:
            self.assertEqual(y, 0, "Divide-by-zero not flagged for y=0")

    @async_test_case
    async def test_division_specific(self, sim):
        """Test specific hand-picked division cases."""
        await self.divide(14, 3)
        await self.divide(14, 7)
        await self.divide(4, 2)
        await self.divide(4, 5)
        await self.divide(12, 13)
        await self.divide(15, 3)
        await self.divide(15, 0)

    @async_test_case
    async def test_division_random(self, sim):
        """Test many random division cases within allowed width."""
        max_val = (1 << self.FRAGMENT_ARGUMENTS["width"]) - 1
        for _ in range(100):
            x = random.randint(0, max_val)
            y = random.randint(0, max_val)
            await self.divide(x, y)
