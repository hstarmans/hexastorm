import unittest

from hexastorm.fpga_host.tools import find_shift


class TestBestCircularShift(unittest.TestCase):
    def test_identical_sequences(self):
        a = [5.000304, 4.998261, 4.998504, 5.002612]
        s, corr, rmse = find_shift(a, a)
        self.assertEqual(s, 0)
        self.assertGreater(corr, 0.99999)
        self.assertAlmostEqual(rmse, 0.0, places=12)

    def test_known_shift_right_by_two(self):
        a = [5.000304, 4.998261, 4.998504, 5.002612]
        # b is a rolled-left-by-2 version of a; to align b to a, roll b right by 2
        b = [4.998504, 5.002612, 5.000304, 4.998261]
        s, corr, rmse = find_shift(a, b)
        self.assertEqual(s, 2)
        self.assertGreater(corr, 0.99999)
        self.assertLess(rmse, 1e-6)

    def test_length_mismatch_raises(self):
        a = [1.0, 2.0, 3.0, 4.0]
        b = [1.0, 2.0, 3.0]
        with self.assertRaises(ValueError):
            find_shift(a, b)
