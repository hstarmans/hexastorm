import sys
import math

# Detect platform
IS_MICROPYTHON = sys.implementation.name != "cpython"

# Platform-specific imports
if IS_MICROPYTHON:
    from ulab import numpy as np
else:
    import numpy as np


def sign(x):
    """Returns the sign of a number or numpy/ulab array.
    -1 for negative, 0 for zero, 1 for positive values.
    """
    if not IS_MICROPYTHON:
        return np.sign(x)
    elif isinstance(x, (int, float)):
        return -1 if x < 0 else 0 if x == 0 else 1
    else:
        # Convert input to ulab array if it's a list
        x = np.array(x) if isinstance(x, list) else x
        result = [(-1 if val < 0 else 0 if val == 0 else 1) for val in x]
        return np.array(result)


def packbits(bitlst, bitorder="big"):
    """Packs a list of binary values into an array of uint8 bytes.
    Args:
        bitlst: list or 1D array of 0s and 1s
        bitorder: "big" or "little" endian bit packing
    Returns:
        Numpy/ulab array of uint8
    """
    if not IS_MICROPYTHON:
        return np.packbits(bitlst, bitorder=bitorder)

    bitlst_len = len(bitlst)
    num_bytes = math.ceil(bitlst_len / 8)
    byte_arr = bytearray(num_bytes)

    for byte_index in range(num_bytes):
        byte_val = 0
        for i in range(8):
            bit_idx = byte_index * 8 + i
            if bit_idx >= bitlst_len:
                break
            bit = bitlst[bit_idx]
            if bitorder == "big":
                byte_val = (byte_val << 1) | bit
            else:  # little-endian
                byte_val |= bit << i
        if bitorder == "big":
            byte_val <<= (8 - min(8, bitlst_len - byte_index * 8))
        byte_arr[byte_index] = byte_val

    return np.array(byte_arr, dtype=np.uint8)



def assert_array_almost_equal(x, y, decimal=6, err_msg="", verbose=True):
    """Assert that two arrays are almost equal up to a given decimal precision."""
    def convert(arr):
        arr = np.array(arr)
        return np.array([0 if math.isnan(v) else v for v in arr])

    x_arr = convert(x)
    y_arr = convert(y)

    tolerance = 1.5 * 10**-decimal
    diffs = np.abs(x_arr - y_arr)

    if not all(d < tolerance for d in diffs):
        message = (
            err_msg or f"Arrays not almost equal to {decimal} decimals:\n{x_arr}\n!=\n{y_arr}"
        )
        raise AssertionError(message)
