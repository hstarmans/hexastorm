"""Dummy ulab module for CPython / development compatibility.

Allows `from ulab import numpy as np` to work in standard Python (CPython/uv)
by redirecting `ulab.numpy` to standard `numpy`.
"""

import numpy as np

numpy = np
