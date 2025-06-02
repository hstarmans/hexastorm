import unittest
from random import randint

import numpy as np

from hexastorm import ulabext


class TestBitPacking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ulabext.IS_MICROPYTHON = True

    def test_packbits_big_and_little(self):
        """Test packbits compatibility for both big and little endian."""
        for bitorder in ("big", "little"):
            for length in range(1, 30):
                with self.subTest(bitorder=bitorder, length=length):
                    # generate random bits
                    bitlist = [randint(0, 1) for _ in range(length)]
                    expected = np.array(np.packbits(bitlist, bitorder=bitorder), dtype=np.uint8)
                    actual = np.array(ulabext.packbits(bitlist, bitorder), dtype=np.uint8)
                    np.testing.assert_array_equal(expected, actual)

    def test_sign_array(self):
        """Test that sign function matches NumPy's output."""
        for length in range(1, 30):
            with self.subTest(length=length):
                # generate random signs
                values = [randint(-1, 1) for _ in range(length)]
                expected = np.array(np.sign(values), dtype=np.int8)
                actual = np.array(ulabext.sign(values), dtype=np.int8)
                np.testing.assert_array_equal(expected, actual)

    def test_assert_array_almost_equal(self):
        """Test compatibility with assert_array_almost_equal."""
        # Identical values with NaN
        a = [1.0, 2.333, np.nan]
        b = [1.0, 2.333, np.nan]
        np.testing.assert_array_almost_equal(a, b)
        ulabext.assert_array_almost_equal(a, b)

        # Values that differ slightly
        a = [1.0, 2.33333, np.nan]
        b = [1.0, 2.33339, np.nan]
        with self.assertRaises(AssertionError):
            np.testing.assert_array_almost_equal(a, b, decimal=5)
        with self.assertRaises(AssertionError):
            ulabext.assert_array_almost_equal(a, b, decimal=5)

        # Values with large mismatch
        a = [1.0, 2.33333, np.nan]
        b = [1.0, 2.33333, 5.0]
        with self.assertRaises(AssertionError):
            np.testing.assert_array_almost_equal(a, b, decimal=5)
        with self.assertRaises(AssertionError):
            ulabext.assert_array_almost_equal(a, b, decimal=5)

