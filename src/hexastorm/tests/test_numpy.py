import unittest
from random import randint

import numpy as np

from ..ulab import packbits


class BitPacking(unittest.TestCase):
    def packbits_test(self, bitorder):
        for i in range(1, 30):
            lst = []
            for _ in range(i):
                lst.append(randint(0, 1))
            a = np.packbits(lst, bitorder=bitorder)
            a = a.astype(np.uint8)
            b = np.array(packbits(lst, bitorder), dtype="uint8")
            np.testing.assert_array_equal(a, b)

    def test_packbits_bit(self):
        self.packbits_test("big")
        self.packbits_test("little")


if __name__ == "__main__":
    unittest.main()
