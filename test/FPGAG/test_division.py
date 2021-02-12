""" test for divider

test script to run on the raspberry pi
"""
import unittest

from luna.gateware.test import LunaGatewareTestCase, sync_test_case

from FPGAG.core import Divisor


class DivisorTest(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = Divisor
    FRAGMENT_ARGUMENTS = {'width':4}

    def do_devision(self, x, y):
        yield self.dut.x.eq(x)
        yield self.dut.y.eq(y)
        yield self.dut.start.eq(1)
        yield
        yield self.dut.start.eq(0)
        while (yield self.dut.valid)==0:
            yield
        self.assertEqual((yield self.dut.q), x//y)
        self.assertEqual((yield self.dut.r), x%y)
        yield self.dut.start.eq(0)
        yield


    @sync_test_case
    def test_division(self):
        # let's set the starting values
        yield from self.do_devision(14, 3)
        yield from self.do_devision(14, 7)
        yield from self.do_devision(4, 2)
        # TODO: doesnt work
        yield from self.do_devision(4,5)
        #yield from self.do_devision(12, 13)
        #yield from self.do_devision(15, 3)

if __name__ == "__main__":
    unittest.main()
