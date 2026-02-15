import logging
import math
from typing import Dict, Tuple, Any

import numpy as np
from numba import jit, typed, types
from hexastorm.config import PlatformConfig, displacement_kernel


config = PlatformConfig(test=False)

# --- JIT Compiled Math Functions ---


displacement = jit(nopython=True, cache=True)(displacement_kernel)


@jit(nopython=True, cache=True)
def fxpos(
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
    dx = params["f" + str(facet_idx) + "_dx"]

    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]

    # Project displacement onto X axis based on polygon tilt
    xpos = (
        (np.sin(params["tiltangle"]) * displacement(line_pixel, params)) + xstart + dx
    )

    # Convert mm to grid pixels
    return xpos / params["samplegridsize"]


@jit(nopython=True, cache=True)
def fypos(
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
    dy = params["f" + str(facet_idx) + "_dy"]

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


# --- Parameter Management ---


def get_default_params(params) -> Any:
    """
    Initializes the system parameters based on the physical configuration.

    Returns:
        A Numba-compatible typed dictionary for high-performance JIT access.
    """
    # Create Numba-compatible dictionary
    dct = typed.Dict.empty(key_type=types.string, value_type=types.float64)
    dct.update(params)

    return dct


def downsample_params(params: Any) -> Any:
    """Adjusts timing parameters if software downsampling is active."""
    # This was used in the beaglebone era, so probably not needed anymore.
    if params["downsamplefactor"] > 1:
        factor = params["downsamplefactor"]
        # Update specific keys
        for key in ["LASER_HZ", "startpixel", "bitsinscanline"]:
            params[key] = round(params[key] / factor)
    return params


# --- Coordinate Generation ---


def calculate_lanewidth(params) -> float:
    """Calculates the physical width [mm] of a single scan lane (one sweep)."""
    return (fxpos(0, params) - fxpos(params["bitsinscanline"] - 1, params)) * params[
        "samplegridsize"
    ]


def calculate_coordinates(params: Any) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Generates the lookup coordinate grid for the entire exposure area.

    This function maps every "tick" of the laser clock to a physical X,Y location
    on the substrate, accounting for:
    1. Prism rotation (Fast Axis / X)
    2. Stage movement (Slow Axis / Y)
    3. Multi-lane stitching (if sample > single scan width)

    Returns:
        ids: (2, N) Array of [Y_pixel, X_pixel] coordinates.
        derived_stats: Dictionary of calculated scan properties.
    """
    if not params["sampleysize"] or not params["samplexsize"]:
        raise ValueError("Sampleysize or samplexsize are set to zero.")

    # Validation: Ensure scan direction is consistent (Start < End physically)
    if fxpos(0, params) < 0 or fxpos(params["bitsinscanline"] - 1, params) > 0:
        raise ValueError("Line seems ill positioned: fxpos(0) < 0 or end > 0")

    # 1. Geometry Planning
    lanewidth = calculate_lanewidth(params)

    # Calculate required lanes to cover X dimension
    lanes = math.ceil(params["samplexsize"] / lanewidth)

    # Calculate required facets (sweeps) to cover Y dimension
    facets_inlane = math.ceil(
        params["rotationfrequency"]
        * params["FACETS"]
        * (params["sampleysize"] / params["stagespeed"])
    )

    logging.info(f"The lanewidth is {lanewidth:.2f} mm")
    logging.info(f"The facets in lane are {facets_inlane}")

    # 2. Base Facet Generation (Simulating a single polygon face sweep)
    pixel_indices = np.arange(int(params["bitsinscanline"]))

    # Center the scanline relative to the lane start
    xstart_val = abs(
        fxpos(int(params["bitsinscanline"]) - 1, params) * params["samplegridsize"]
    )

    y_shift_per_facet = (params["stagespeed"]) / (
        params["FACETS"] * params["rotationfrequency"] * params["samplegridsize"]
    )

    # x_width_pixels for lane stitching
    x_width_pixels = fxpos(0, params, 0) - fxpos(
        params["bitsinscanline"] - 1, params, 0
    )

    # 3. Coordinate Generation
    # We will accumulate sweeps in lists then stack once for performance
    all_x = []
    all_y = []

    num_physical_facets = int(params["FACETS"])

    for l_idx in range(lanes):
        is_forward = l_idx % 2 == 0
        # Horizontal offset for this lane (stitching)
        lane_x_offset = l_idx * x_width_pixels

        for f_lane_idx in range(facets_inlane):
            # IMPORTANT: Calculate which physical facet on the prism is active
            # This follows your modulo logic: facet 0, 1, 2, 3, 0, 1...
            facet_idx = f_lane_idx % num_physical_facets

            # Calculate stage movement for this specific sweep
            # For forward lanes, stage goes 0 -> end. For backward, end -> 0.
            if is_forward:
                ystart_mm = f_lane_idx * y_shift_per_facet * params["samplegridsize"]
            else:
                ystart_mm = (
                    (facets_inlane - f_lane_idx)
                    * y_shift_per_facet
                    * params["samplegridsize"]
                )

            # Compute corrected X and Y for this entire sweep
            # These are now unique per facet!
            x_sweep = fxpos(pixel_indices, params, facet_idx, xstart_val)
            y_sweep = fypos(pixel_indices, params, facet_idx, is_forward, ystart_mm)

            all_x.append((x_sweep + lane_x_offset).astype(np.int32))
            all_y.append(y_sweep.astype(np.int32))

    # 4. Final Assembly
    # Concatenate the lists of arrays into single flat coordinate vectors
    x_final = np.concatenate(all_x)
    y_final = np.concatenate(all_y)

    ids = np.vstack((x_final, y_final)).astype(np.int32)

    derived_stats = {
        "lanewidth": lanewidth,
        "facetsinlane": facets_inlane,
        "lanes": lanes,
    }

    return ids, derived_stats
