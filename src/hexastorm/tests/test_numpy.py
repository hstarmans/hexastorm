import unittest
from random import randint

import numpy as np

from .. import ulabext


class BitPacking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ulabext.micropython = True

    def packbits_test(self, bitorder):
        for i in range(1, 30):
            lst = []
            for _ in range(i):
                lst.append(randint(0, 1))
            a = np.packbits(lst, bitorder=bitorder)
            a = a.astype(np.uint8)
            b = np.array(ulabext.packbits(lst, bitorder), dtype="uint8")
            np.testing.assert_array_equal(a, b)

    def test_packbits_bit(self):
        self.packbits_test("big")
        self.packbits_test("little")

    def test_sign(self):
        for i in range(1, 30):
            lst = []
            for _ in range(i):
                lst.append(randint(-1, 1))
        a = np.sign(np.array(lst))
        a = a.astype(np.uint8)
        b = np.array(ulabext.sign(np.array(lst)), dtype="uint8")
        np.testing.assert_array_equal(a, b)

    def test_assert_array_almost_equal(self):
        a, b = [1.0, 2.333, np.nan], [1.0, 2.333, np.nan]
        np.testing.assert_array_almost_equal(a, b)
        ulabext.assert_array_almost_equal(a, b)
        a, b = [1.0, 2.33333, np.nan], [1.0, 2.33339, np.nan]
        with self.assertRaises(AssertionError):
            np.testing.assert_array_almost_equal(a, b, decimal=5)
        with self.assertRaises(AssertionError):
            ulabext.assert_array_almost_equal(a, b, decimal=5)
        a, b = [1.0, 2.33333, np.nan], [1.0, 2.33333, 5]
        with self.assertRaises(AssertionError):
            np.testing.assert_array_almost_equal(a, b, decimal=5)
        with self.assertRaises(AssertionError):
            ulabext.assert_array_almost_equal(a, b, decimal=5)


if __name__ == "__main__":
    unittest.main()
