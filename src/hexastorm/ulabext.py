import sys
import math


micropython = True
if sys.implementation.name == "cpython":
    import numpy as np

    micropython = False
else:
    from ulab import numpy as np


def sign(array):
    """Copy of sign for ulab"""
    if not micropython:
        return np.sign(array)
    else:
        singlevalue = False
        if not isinstance(array, (np.array, list)):
            singlevalue = True
            array = [array]
        for idx, val in enumerate(array):
            if val < 0:
                array[idx] = -1
            elif val == 0:
                array[idx] = 0
            else:
                array[idx] = 1
        if singlevalue:
            return array[0]
        else:
            return array


def packbits(bitlst, bitorder="big"):
    """Packs elements of a binary-valued array
    into bits in a uint8 array."""
    if not micropython:
        return np.packbits(bitlst, bitorder=bitorder)
    else:
        tmp_str = ""
        byte_lst = np.zeros(math.ceil(len(bitlst) / 8), dtype=np.uint8)

        cnt = 0
        for idx, bit in enumerate(bitlst):
            tmp_str += str(bit)
            if len(tmp_str) == 8:
                if bitorder == "little":
                    # not supported in micropython
                    # tmp_str = tmp_str[::-1]
                    tmp_str = "".join(reversed(tmp_str))
                byte_lst[cnt] = int(tmp_str, 2)
                tmp_str = ""
                cnt += 1

        if len(tmp_str) > 0:
            tmp_str += "0" * (8 - len(tmp_str))
            if bitorder == "little":
                tmp_str = "".join(reversed(tmp_str))
            byte_lst[cnt] = int(tmp_str, 2)
        return byte_lst


def assert_array_almost_equal(x, y, decimal=6, err_msg="", verbose=True):
    def conversion(x):
        return np.array([0 if math.isnan(a) else a for a in x])

    x, y = conversion(x), conversion(y)
    if not all(abs(x - y) < (1.5 * 10 ** (-decimal))):
        raise AssertionError(
            f"{x} and {y} are not almost equal up to {decimal}"
        )
