import math
import sys

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
            byte_val <<= 8 - min(8, bitlst_len - byte_index * 8)
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
    # np.abs does not exist in micropython
    diffs = abs(x_arr - y_arr)

    if not all(d < tolerance for d in diffs):
        message = (
            err_msg
            or f"Arrays not almost equal to {decimal} decimals:\n{x_arr}\n!=\n{y_arr}"
        )
        raise AssertionError(message)


SENTINEL_TICK = 0xFFFFFF


def calc_step_half_period(steps_line: float, scanline_ticks: int) -> int:
    """Compute orthogonal-stage step half-period in FPGA clock ticks (0 = stationary)."""
    if steps_line <= 0:
        return 0
    half_period = math.floor((scanline_ticks / (steps_line * 2.0)) + 1e-9) - 1
    if half_period < 1:
        raise ValueError("Steps per line cannot be achieved (period < 1)")
    return int(half_period)


def bits_to_events(laser_bits, scanline_ticks: int) -> list[tuple[int, int]]:
    """Convert a 1D binary sequence into (tick, intensity) transitions over ``[0, scanline_ticks)``.

    Assumes the laser starts OFF (0) at tick 0 of the active scanline window.
    """
    total_ticks = int(scanline_ticks)
    n = len(laser_bits)
    if n == 0:
        return []
    if n > total_ticks:
        raise ValueError(
            f"laser_bits length ({n}) exceeds scanline_ticks ({total_ticks})"
        )

    if not IS_MICROPYTHON and isinstance(laser_bits, np.ndarray):
        arr = np.asarray(laser_bits, dtype=np.uint8) != 0
        prev = np.empty_like(arr)
        prev[0] = False
        prev[1:] = arr[:-1]
        change_idx = np.flatnonzero(arr != prev)
        if change_idx.size == 0:
            return []
        ticks = ((change_idx.astype(np.int64) * total_ticks) + n - 1) // n
        intensities = np.where(arr[change_idx], 255, 0)
        return list(zip(ticks.tolist(), intensities.tolist()))

    events = []
    state = False
    for idx, bit in enumerate(laser_bits):
        b = bool(bit)
        if b != state:
            tick = ((idx * total_ticks) + n - 1) // n
            events.append((tick, 255 if b else 0))
            state = b
    return events


def events_to_bits(
    events: list[tuple[int, int]], num_bits: int, scanline_ticks: int
) -> list[int]:
    """Reconstruct a binary 0/1 list of length ``num_bits`` from FPGA tick events."""
    n = int(num_bits)
    total_ticks = int(scanline_ticks)
    bits = [0] * n
    if not events or n <= 0 or total_ticks <= 0:
        return bits

    valid_events = [(int(t), int(v)) for t, v in events if 0 <= int(t) < SENTINEL_TICK]
    for i, (tick, intensity) in enumerate(valid_events):
        start_px = max(0, (tick * n) // total_ticks)
        if start_px >= n:
            break
        if i + 1 < len(valid_events):
            end_px = min(n, (valid_events[i + 1][0] * n) // total_ticks)
        else:
            end_px = n
        bit_val = 1 if intensity > 0 else 0
        for px in range(start_px, end_px):
            bits[px] = bit_val
    return bits


def pack_scanline_events(
    events: list[tuple[int, int]],
    steps_line: float,
    direction: int,
    scanline_ticks: int,
    scanline_cmd: int = 3,
) -> bytearray:
    """Pack a scanline header and its (tick, intensity) events into 8-byte aligned bytes.

    Word 0 (8 bytes, little-endian payload):
        bits  0..7 : scanline_cmd (Spi.Instructions.scanline = 0x03)
        bit   8    : direction (0 or 1)
        bits  9..24: event_words (16-bit count of 64-bit event words following header)
        bits 25..63: step half_period in FPGA ticks (39-bit integer; 0 = stationary)

    Word 1..event_words (8 bytes each, two 32-bit events per 64-bit word):
        ev0 (bits  0..31): [24-bit tick << 8 | 8-bit intensity]
        ev1 (bits 32..63): [24-bit tick << 8 | 8-bit intensity] or 0xFFFFFF00 sentinel
    """
    half_period = calc_step_half_period(steps_line, scanline_ticks)

    num_events = len(events)
    event_words = (num_events + 1) // 2
    if event_words > 0xFFFF:
        raise ValueError(f"Too many events in scanline: {num_events}")

    payload_int = (
        ((half_period & ((1 << 39) - 1)) << 17)
        | ((event_words & 0xFFFF) << 1)
        | (int(direction) & 1)
    )
    out = bytearray()
    out.append(scanline_cmd & 0xFF)
    out.extend(payload_int.to_bytes(7, "little"))

    for w_idx in range(event_words):
        t0, v0 = events[2 * w_idx]
        ev0 = ((int(t0) & 0xFFFFFF) << 8) | (int(v0) & 0xFF)
        if 2 * w_idx + 1 < num_events:
            t1, v1 = events[2 * w_idx + 1]
            ev1 = ((int(t1) & 0xFFFFFF) << 8) | (int(v1) & 0xFF)
        else:
            ev1 = SENTINEL_TICK << 8
        word64 = ev0 | (ev1 << 32)
        out.extend(word64.to_bytes(8, "little"))

    return out
