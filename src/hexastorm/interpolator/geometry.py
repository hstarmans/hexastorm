import logging
import math
from typing import Dict, Tuple, Any

import numpy as np
from numba import jit, typed, types

from ..config import displacement_kernel

# --- JIT Compiled Math Functions ---

# We compile the kernel independently because it is a pure mathematical function
# that doesn't depend on the class state, just the params passed to it.
displacement = jit(nopython=True, cache=True)(displacement_kernel)

logger = logging.getLogger(__name__)


@jit(nopython=True, cache=True)
def _jit_fxpos(
    pixel: float, params: Dict[str, float], facet_idx: int = 0, xstart: float = 0
) -> float:
    """
    Returns the laser diode X-position (Transversal / Scan Direction) in pixels.

    The X-axis typically represents the direction the laser scans across the sample.
    If tiltangle (alpha) is 90 degrees (Hexastorm standard), sin(90)=1, so
    X receives 100% of the prism displacement. AMSystems/LDIsystems/KleoLDI use
    different angles.

    Args:
        pixel: The pixel index.
        params: System parameters.
        facet_idx: Index of the current facet.
        xstart: Offset in mm.
    """
    # Get the correction for the current facet
    dx = params["f" + str(facet_idx) + "_scan"]

    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]

    # Project displacement onto X axis based on polygon tilt
    xpos = (
        (np.sin(params["tiltangle"]) * displacement(line_pixel, params)) + xstart + dx
    )

    # Convert mm to grid pixels
    return xpos / params["samplegridsize"]


@jit(nopython=True, cache=True)
def _jit_fypos(
    pixel: float,
    params: Dict[str, float],
    facet_idx: int,
    direction: bool,
    ystart: float = 0,
) -> float:
    """
    Returns the laser diode Y-position (Longitudinal / Stage Direction) in pixels.

    The Y-axis is the "slow" axis parallel to the stage movement.
    The position is a sum of two components:
    1. Projection of Prism Displacement: If tiltangle != 90, some scan motion bleeds into Y.
    2. Stage Movement: The continuous motion of the substrate (v = stagespeed).

    Args:
        pixel: The pixel index.
        params: System parameters.
        direction: Boolean indicating scan direction (Zig-Zag scanning).
        facet_idx: Index of the current facet.
        ystart: Y-offset in mm.
    """
    dy = params["f" + str(facet_idx) + "_orth"]

    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]

    # 1. Optical Component: Projection of displacement onto Y (usually 0 if tilt=90)
    base_y = (-np.cos(params["tiltangle"]) * displacement(line_pixel, params)) + dy

    # 2. Mechanical Component: Stage movement over time
    # time = pixel / LASER_HZ
    # distance = time * stagespeed
    stage_component = (line_pixel / params["LASER_HZ"]) * params["stagespeed"]

    if direction:
        ypos = base_y + stage_component + ystart
    else:
        ypos = base_y - stage_component + ystart

    return ypos / params["samplegridsize"]


@jit(nopython=True, cache=True)
def _jit_calculate_grid(params: Any) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Generates the lookup coordinate grid using pre-allocation to avoid
    Numba concatenation errors.
    """
    if not params["sampleysize"] or not params["samplexsize"]:
        raise ValueError("Sampleysize or samplexsize are set to zero.")

    # Validation
    if (
        _jit_fxpos(0, params, 0, 0.0) < 0
        or _jit_fxpos(params["bitsinscanline"] - 1, params, 0, 0.0) > 0
    ):
        raise ValueError("Line seems ill positioned: fxpos(0) < 0 or end > 0")

    # 1. Geometry Planning
    start_x = _jit_fxpos(0.0, params, 0, 0.0)
    end_x = _jit_fxpos(params["bitsinscanline"] - 1, params, 0, 0.0)
    lanewidth = (start_x - end_x) * params["samplegridsize"]

    # Calculate required lanes
    lanes = math.ceil(params["samplexsize"] / lanewidth)

    # Calculate required facets
    facets_inlane = math.ceil(
        params["rotationfrequency"]
        * params["FACETS"]
        * (params["sampleysize"] / params["stagespeed"])
    )

    # --- PRE-CALCULATE SIZE & ALLOCATE MEMORY ---
    # This replaces the dynamic list appending
    points_per_sweep = int(params["bitsinscanline"])
    total_points = int(lanes * facets_inlane * points_per_sweep)

    # Pre-allocate the final arrays with the exact size needed
    x_final = np.empty(total_points, dtype=np.int32)
    y_final = np.empty(total_points, dtype=np.int32)

    # 2. Base Facet Generation
    pixel_indices = np.arange(points_per_sweep)

    xstart_val = abs(
        _jit_fxpos(points_per_sweep - 1, params) * params["samplegridsize"]
    )

    y_shift_per_facet = (params["stagespeed"]) / (
        params["FACETS"] * params["rotationfrequency"] * params["samplegridsize"]
    )

    x_width_pixels = _jit_fxpos(0, params, 0) - _jit_fxpos(
        points_per_sweep - 1, params, 0
    )

    # 3. Coordinate Generation
    num_physical_facets = int(params["FACETS"])

    # Track where we are inserting data into the final arrays
    current_idx = 0

    for l_idx in range(lanes):
        is_forward = l_idx % 2 == 0
        lane_x_offset = l_idx * x_width_pixels

        for f_lane_idx in range(facets_inlane):
            facet_idx = f_lane_idx % num_physical_facets

            if is_forward:
                ystart_mm = f_lane_idx * y_shift_per_facet * params["samplegridsize"]
            else:
                ystart_mm = (
                    (facets_inlane - f_lane_idx)
                    * y_shift_per_facet
                    * params["samplegridsize"]
                )

            # Calculate sweep
            x_sweep = _jit_fxpos(pixel_indices, params, facet_idx, xstart_val)
            y_sweep = _jit_fypos(
                pixel_indices, params, facet_idx, is_forward, ystart_mm
            )

            # --- DIRECT INSERTION ---
            # Instead of append, we slice directly into the final array
            end_idx = current_idx + points_per_sweep

            x_final[current_idx:end_idx] = (x_sweep + lane_x_offset).astype(np.int32)
            y_final[current_idx:end_idx] = y_sweep.astype(np.int32)

            # Move the pointer forward
            current_idx = end_idx

    # 4. Final Assembly
    ids = np.vstack((x_final, y_final)).astype(np.int32)

    params["lanewidth"] = lanewidth
    params["lanes"] = float(lanes)
    params["facetsinlane"] = float(facets_inlane)

    return ids


class ScannerModel:
    """
    Standard Python wrapper around JIT-compiled kernels.
    Holds the state (params) and delegates calculation to Numba functions.
    """

    def __init__(self, params):
        self.params = self.to_numba_dict(params)

    @staticmethod
    def to_numba_dict(std_dict: Dict[str, Any]) -> Any:
        """
        Converts a standard Python dictionary to a Numba-compatible typed dictionary.
        This enables JIT compilation for functions using this data.
        """
        dct = typed.Dict.empty(key_type=types.string, value_type=types.float64)
        for k, v in std_dict.items():
            dct[k] = float(v)
        return dct

    def calculate_coordinates(self) -> np.ndarray:
        """
        Generates the lookup coordinate grid for the entire exposure area.

        This function maps every "tick" of the laser clock to a physical X,Y location
        on the substrate, accounting for:
        1. Prism rotation (Fast Axis / X)
        2. Stage movement (Slow Axis / Y)
        3. Multi-lane stitching (if sample > single scan width)

        Returns:
            ids: (2, N) Array of [Y_pixel, X_pixel] coordinates.
        """
        ids = _jit_calculate_grid(self.params)
        logger.debug(f"The lanewidth is {self.params['lanewidth']:.2f} mm")
        logger.debug(f"The facets in lane are {int(self.params['facetsinlane'])}")
        return ids
