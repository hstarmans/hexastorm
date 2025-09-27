try:
    import numpy as np
except ImportError:
    from ulab import numpy as np


def find_shift(a, b):
    """
    Find shift s so that roll(b, s) best matches a.
    Returns (s, corr, rmse).

    - s: integer shift in [0, n-1], right-roll on b
    - corr: normalized correlation at that shift ([-1, 1])
    - rmse: root-mean-square error after applying the shift
    """
    a = np.array(a)
    b = np.array(b)
    if a.shape[0] != b.shape[0]:
        raise ValueError("a and b must have same length")

    # remove DC to make correlation robust to offsets
    a0 = a - np.mean(a)
    b0 = b - np.mean(b)

    # normalization for correlation
    denom = np.sqrt(np.dot(a0, a0) * np.dot(b0, b0))
    if denom == 0:
        # handle constant or near-constant signals
        # fall back to minimizing RMSE over shifts
        denom = 1.0

    n = a.shape[0]
    best_s = 0
    best_corr = -1e9

    for s in range(n):
        br = np.roll(b0, s)  # candidate alignment
        num = np.dot(a0, br)
        corr = num / denom
        if corr > best_corr:
            best_corr = corr
            best_s = s

    # quality metric at the chosen shift
    br_full = np.roll(b, best_s)
    rmse = np.sqrt(np.mean((a - br_full) ** 2))

    return int(best_s), float(best_corr), float(rmse)
