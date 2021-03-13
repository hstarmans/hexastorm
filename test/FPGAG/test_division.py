import unittest
import random

from luna.gateware.test import LunaGatewareTestCase, sync_test_case

from FPGAG.notused import Divisor, Multiplication


class MultiplicationTest(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = Multiplication

    @sync_test_case
    def test_multiply(self):
        a = 319
        b = 40
        yield self.dut.a.eq(a)
        yield self.dut.b.eq(b)
        yield
        yield
        self.assertEqual((yield self.dut.c), a*b)


class DivisorTest(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = Divisor
    FRAGMENT_ARGUMENTS = {'width': 6}

    def do_devision(self, x, y):
        bits = self.FRAGMENT_ARGUMENTS['width']
        assert x.bit_length() <= bits
        assert y.bit_length() <= bits
        yield self.dut.x.eq(x)
        yield self.dut.y.eq(y)
        yield self.dut.start.eq(1)
        yield
        yield self.dut.start.eq(0)
        yield
        while (yield self.dut.valid) == 0 |
               (yield self.dut.dbz) == 0):
            yield
        if (yield self.dut.valid)==1:
            self.assertEqual((yield self.dut.q), x//y)
            self.assertEqual((yield self.dut.r), x%y)
        else:
            self.assertEqual(y, 0)

    @sync_test_case
    def test_division(self):
        yield from self.do_devision(14, 3)
        yield from self.do_devision(14, 7)
        yield from self.do_devision(4, 2)
        yield from self.do_devision(4, 5)
        yield from self.do_devision(12, 13)
        yield from self.do_devision(15, 3)
        yield from self.do_devision(15, 0)

    @sync_test_case
    def test_random(self):
        maxint = int('1'*self.FRAGMENT_ARGUMENTS['width'], 2)
        x = random.randint(0, maxint)
        y = random.randint(0, maxint)
        for _ in range(100):
            yield from self.do_devision(x, y)


if __name__ == "__main__":
    unittest.main()
