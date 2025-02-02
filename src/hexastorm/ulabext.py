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
        if isinstance(array, (int, float)):  # Handle single values directly
            return -1 if array < 0 else 0 if array == 0 else 1

        for idx, val in enumerate(array):
            if val < 0:
                array[idx] = -1
            elif val == 0:
                array[idx] = 0
            else:
                array[idx] = 1
        return array


def packbits(bitlst, bitorder="big"):
    """Packs elements of a binary-valued array
    into bits in a uint8 array."""
    if not micropython:
        return np.packbits(bitlst, bitorder=bitorder)
    else:
        bitlst_len = len(bitlst)
        num_bytes = math.ceil(bitlst_len / 8)
        byte_arr = bytearray(num_bytes)  # Pre-allocate with correct size
        byte_index = 0
        bit_index = 0

        if bitorder == "big":
            while bit_index < bitlst_len:
                byte_value = 0
                remaining_bits = min(8, bitlst_len - bit_index) # calculate remaining bits
                for i in range(remaining_bits):
                    byte_value = (byte_value << 1) | bitlst[bit_index]
                    bit_index += 1
                byte_arr[byte_index] = byte_value << (8 - remaining_bits) # pad with zeros
                byte_index += 1

        else:  # little-endian
            while bit_index < bitlst_len:
                byte_value = 0
                remaining_bits = min(8, bitlst_len - bit_index) # calculate remaining bits
                for i in range(remaining_bits):
                    byte_value |= bitlst[bit_index] << i
                    bit_index += 1
                byte_arr[byte_index] = byte_value
                byte_index += 1

        return np.array(byte_arr, dtype=np.uint8)


def assert_array_almost_equal(x, y, decimal=6, err_msg="", verbose=True):
    def conversion(x):
        return np.array([0 if math.isnan(a) else a for a in x])

    x, y = conversion(x), conversion(y)
    if not all(abs(x - y) < (1.5 * 10 ** (-decimal))):
        raise AssertionError(
            f"{x} and {y} are not almost equal up to {decimal}"
        )
