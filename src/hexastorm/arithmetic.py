# code was built as exercise, it is used nowhere in the project
# should probably be removed
import random
import unittest

from amaranth import Cat, Const, Elaboratable, Module, Signal, signed

from .utils import LunaGatewareTestCase, async_test_case


class Multiplication(Elaboratable):
    def __init__(self):
        self.a = Signal(signed(16))
        self.b = Signal(signed(16))
        self.c = Signal(signed(32))

    def elaborate(self, platform):
        m = Module()
        m.d.sync += [self.c.eq(self.a * self.b)]
        return m


class Divisor(Elaboratable):
    """Euclidean division with a remainder

    X = Y*Q + R
    dividend X by divisor Y you get quotient Q and remainder R
    For a tutorial see https://projectf.io/posts/division-in-verilog/

        width            -- number of bits of divisor and quotients
    I/O signals:
        I: start         -- calculation starts on high
        O: busy          -- calculation in progress
        O: valid         -- result is valid
        O: dbz           -- divide by zero
        I: x             -- dividend
        I: y             -- divisor
        O: q             -- quotients
        O: r             -- remainder
    """

    def __init__(self, width=4):
        self.width = width
        self.start = Signal()
        self.busy = Signal()
        self.valid = Signal()
        self.dbz = Signal()
        self.x = Signal(width)
        self.y = Signal(width)
        self.q = Signal(width)
        self.r = Signal(width)

    def elaborate(self, platform):
        m = Module()
        ac = Signal(self.width + 1)
        ac_next = Signal.like(ac)
        temp = Signal.like(ac)
        q1 = Signal(self.width)
        q1_next = Signal.like(q1)
        i = Signal(range(self.width))
        # combinatorial
        with m.If(ac >= self.y):
            m.d.comb += [
                temp.eq(ac - self.y),
                Cat(q1_next, ac_next).eq(Cat(1, q1, temp[0 : self.width - 1])),
            ]
        with m.Else():
            m.d.comb += [Cat(q1_next, ac_next).eq(Cat(q1, ac) << 1)]
        # synchronized
        with m.If(self.start):
            m.d.sync += [self.valid.eq(0), i.eq(0)]
            with m.If(self.y == 0):
                m.d.sync += [self.busy.eq(0), self.dbz.eq(1)]
            with m.Else():
                m.d.sync += [
                    self.busy.eq(1),
                    self.dbz.eq(0),
                    Cat(q1, ac).eq(
                        Cat(Const(0, 1), self.x, Const(0, self.width))
                    ),
                ]
        with m.Elif(self.busy):
            with m.If(i == self.width - 1):
                m.d.sync += [
                    self.busy.eq(0),
                    self.valid.eq(1),
                    i.eq(0),
                    self.q.eq(q1_next),
                    self.r.eq(ac_next >> 1),
                ]
            with m.Else():
                m.d.sync += [i.eq(i + 1), ac.eq(ac_next), q1.eq(q1_next)]
        return m


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


if __name__ == "__main__":
    unittest.main()
