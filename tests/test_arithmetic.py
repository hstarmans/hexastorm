import random
import unittest

from hexastorm.arithmetic import Multiplication, Divisor
from hexastorm.utils import LunaGatewareTestCase, async_test_case

class MultiplicationTest(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = Multiplication

    @async_test_case
    async def test_multiply(self, sim):
        a = 319
        b = 40

        sim.set(self.dut.a, a)
        sim.set(self.dut.b, b)

        await sim.tick()
        await sim.tick()  # Allow for propagation/delay if needed

        self.assertEqual(sim.get(self.dut.c), a * b)


class DivisorTest(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = Divisor
    FRAGMENT_ARGUMENTS = {"width": 6}

    async def do_division(self, sim, x, y):
        bits = self.FRAGMENT_ARGUMENTS["width"]
        assert x.bit_length() <= bits
        assert y.bit_length() <= bits

        sim.set(self.dut.x, x)
        sim.set(self.dut.y, y)
        sim.set(self.dut.start, 1)
        await sim.tick()
        sim.set(self.dut.start, 0)
        await sim.tick()

        # Wait for valid output or DBZ
        for _ in range(100):  # Add a timeout to avoid infinite loop
            if sim.get(self.dut.valid) or sim.get(self.dut.dbz):
                break
            await sim.tick()
        else:
            self.fail("Division did not complete within expected cycles")

        if sim.get(self.dut.valid):
            self.assertEqual(sim.get(self.dut.q), x // y)
            self.assertEqual(sim.get(self.dut.r), x % y)
        else:
            self.assertEqual(y, 0)

    @async_test_case
    async def test_division(self, sim):
        await self.do_division(sim, 14, 3)
        await self.do_division(sim, 14, 7)
        await self.do_division(sim, 4, 2)
        await self.do_division(sim, 4, 5)
        await self.do_division(sim, 12, 13)
        await self.do_division(sim, 15, 3)
        await self.do_division(sim, 15, 0)

    @async_test_case
    async def test_random(self, sim):
        maxint = int("1" * self.FRAGMENT_ARGUMENTS["width"], 2)
        for _ in range(100):
            x = random.randint(0, maxint)
            y = random.randint(0, maxint)
            await self.do_division(sim, x, y)