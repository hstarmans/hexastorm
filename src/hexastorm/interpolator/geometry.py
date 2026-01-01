import logging
import math
from typing import Dict, Tuple, Any

import numpy as np
from numba import jit, typed, types
from hexastorm.config import PlatformConfig

# --- JIT Compiled Math Functions ---


@jit(nopython=True, cache=True)
def displacement(pixel: float, params: Dict[str, float]) -> float:
    """returns the displacement for a given pixel

    The x-axis is parallel to the scanline if the stage does not move.
    It is assumed, the laser bundle traverses
    in the negative direction and the prism motor
    rotates counterclockwise.

    pixel  --  the pixelnumber
    params --  numba typed dictionary
    """
    # interiorangle = 180-360/self.n
    # max_angle = 180-90-0.5*interiorangle
    max_angle = 180 / params["FACETS"]
    pixelsfacet = round(
        params["LASER_HZ"] / (params["rotationfrequency"] * params["FACETS"])
    )
    angle = np.radians(-2 * max_angle * (pixel / pixelsfacet) + max_angle)

    # Pre-calculate sin(angle) for readability and slight perf
    sin_angle = np.sin(angle)
    sin_angle_sq = np.power(sin_angle, 2)
    n_sq = np.power(params["n"], 2)

    disp = (
        params["inradius"]
        * 2
        * sin_angle
        * (1 - np.power((1 - sin_angle_sq) / (n_sq - sin_angle_sq), 0.5))
    )
    return disp


@jit(nopython=True, cache=True)
def fxpos(pixel: float, params: Dict[str, float], xstart: float = 0) -> float:
    """returns the laserdiode x-position in pixels

    The x-axis is parallel to the scanline if the stage does not move.
    pixel   -- the pixelnumber in the line
    xstart  -- the x-start position [mm], typically
                your xstart is larger than 0 as the displacement
                can be negative
    """
    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]
    xpos = np.sin(params["tiltangle"]) * displacement(line_pixel, params) + xstart
    return xpos / params["samplegridsize"]


@jit(nopython=True, cache=True)
def fypos(
    pixel: float, params: Dict[str, float], direction: bool, ystart: float = 0
) -> float:
    """
    returns the laserdiode y-position in pixels

    The y-axis is orthogonal to the scanline if the stage does not move,
    and parallel to the stage movement.
    pixel      -- the pixelnumber in the line
    direction  -- True is +, False is -
    ystart     -- the y-start position [mm]
    """
    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]

    # Calculate base Y position from prism rotation
    # Note: displacement is calculated same for fwd/bwd, but scanning direction differs?
    # Original logic preserved:
    base_y = -np.cos(params["tiltangle"]) * displacement(line_pixel, params)

    # Calculate stage movement component
    stage_component = (line_pixel / params["LASER_HZ"]) * params["stagespeed"] + ystart

    if direction:
        ypos = base_y + stage_component
    else:
        ypos = base_y - stage_component

    return ypos / params["samplegridsize"]


# --- Parameter Management ---


def get_default_params(stepsperline: int = 1, config: PlatformConfig = None) -> Any:
    """
    Creates the Numba-compatible typed dictionary with all geometry parameters.
    """
    if config is None:
        config = PlatformConfig(test=False)

    laz_tim = config.laser_timing
    mtr_cfg = config.motor_cfg

    # Create Numba-compatible dictionary
    dct = typed.Dict.empty(key_type=types.string, value_type=types.float64)

    py_dict = {
        # angle [radians]
        "tiltangle": np.radians(90),
        "LASER_HZ": laz_tim["laser_hz"],  # Hz
        # rotation frequency polygon [Hz]
        "rotationfrequency": laz_tim["rpm"] / 60,
        # number of facets
        "FACETS": laz_tim["facets"],
        # inradius polygon [mm]
        "inradius": 15,
        # refractive index
        "n": 1.49,
        # platform size scanning direction [mm]
        "pltfxsize": 200,
        # platform size stacking direction [mm]
        "pltfysize": 200,
        # sample size scanning direction [mm]
        "samplexsize": 0.0,
        # sample size stacking direction [mm]
        "sampleysize": 0.0,
        # height/width of the sample grid [mm]
        "samplegridsize": 0.01,
        "stepsperline": float(stepsperline),
        "facetsinlane": 0.0,  # set during calculation
        # mm/s
        "stagespeed": (
            (stepsperline / mtr_cfg["steps_mm"][mtr_cfg["orth2lsrline"]])
            * (laz_tim["rpm"] / 60)
            * laz_tim["facets"]
        ),
        # first calculates all bits in scanline and then the start
        "startpixel": (
            (laz_tim["scanline_length"] / (laz_tim["end_frac"] - laz_tim["start_frac"]))
            * laz_tim["start_frac"]
        ),
        # number of pixels in a line
        "bitsinscanline": laz_tim["scanline_length"],
        "downsamplefactor": 1.0,
    }

    dct.update(py_dict)

    # Calculate initial lanewidth (pure geometry)
    dct["lanewidth"] = (fxpos(0, dct) - fxpos(dct["bitsinscanline"] - 1, dct)) * dct[
        "samplegridsize"
    ]

    return dct


def downsample_params(params: Any) -> Any:
    """Adjusts parameters if downsamplefactor > 1"""
    if params["downsamplefactor"] > 1:
        factor = params["downsamplefactor"]
        # Update specific keys
        for key in ["LASER_HZ", "startpixel", "bitsinscanline"]:
            params[key] = round(params[key] / factor)
    return params


# --- Coordinate Generation ---


def calculate_lanewidth(params) -> float:
    """Helper to calculate lane width in mm based on current params"""
    return (fxpos(0, params) - fxpos(params["bitsinscanline"] - 1, params)) * params[
        "samplegridsize"
    ]


def calculate_coordinates(params: Any) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Calculates the X, Y position of the laser diode for each pixel.

    Returns:
        ids: (2, N) numpy array of coordinates [y, x]
        derived_stats: Dictionary containing calculated 'lanewidth' and 'facetsinlane'
    """
    if not params["sampleysize"] or not params["samplexsize"]:
        raise ValueError("Sampleysize or samplexsize are set to zero.")

    # Check if line positioning is valid
    if fxpos(0, params) < 0 or fxpos(params["bitsinscanline"] - 1, params) > 0:
        raise ValueError("Line seems ill positioned: fxpos(0) < 0 or end > 0")

    # 1. Basic Geometry Calculation
    lanewidth = calculate_lanewidth(params)

    lanes = math.ceil(params["samplexsize"] / lanewidth)
    facets_inlane = math.ceil(
        params["rotationfrequency"]
        * params["FACETS"]
        * (params["sampleysize"] / params["stagespeed"])
    )

    logging.info(f"The lanewidth is {lanewidth:.2f} mm")
    logging.info(f"The facets in lane are {facets_inlane}")

    # 2. Base Facet Generation (Single scanline)
    # We pass the entire range of pixels to the JIT functions at once
    pixel_indices = np.arange(int(params["bitsinscanline"]))

    # Calculate xstart (mm to pixel conversion)
    xstart_val = abs(
        fxpos(int(params["bitsinscanline"]) - 1, params) * params["samplegridsize"]
    )

    # Numba JIT calls
    xpos_facet = fxpos(pixel_indices, params, xstart_val).astype(np.int16)
    ypos_fwd_facet = fypos(pixel_indices, params, True)
    ypos_bwd_facet = fypos(pixel_indices, params, False)

    # 3. Facet Offsets
    # Create an array of offsets for every facet in a lane
    facet_idx = np.arange(facets_inlane).reshape(-1, 1)  # Shape (facets, 1)

    # Shift factor: movement of stage per facet
    y_shift_per_facet = (params["stagespeed"]) / (
        params["FACETS"] * params["rotationfrequency"] * params["samplegridsize"]
    )

    # Broadcast (facets, 1) + (1, pixels) -> (facets, pixels)
    y_fwd_lane = (ypos_fwd_facet + (facet_idx * y_shift_per_facet)).flatten()
    y_bwd_lane = (
        ypos_bwd_facet + ((facets_inlane - facet_idx) * y_shift_per_facet)
    ).flatten()
    x_lane_base = np.tile(xpos_facet, facets_inlane)

    # 4. Lane Offsets (Broadcasting)
    lane_idx = np.arange(lanes).reshape(-1, 1)  # Shape (lanes, 1)

    # Calculate pure pixel width of a lane for offset calculation
    # Note: Re-calculating using fxpos to ensure consistency
    x_width_mm = fxpos(0, params) - fxpos(params["bitsinscanline"] - 1, params)
    # x_width_pixel is effectively lane width in derived pixels?
    # The original code did: x_width_pixel = ...
    # Wait, the original code used 'x_width_pixel' but passed it to x_offsets
    # It seems to be simply params['lanewidth'] / gridsize, but let's stick to the math:
    x_width_val = (
        x_width_mm  # This is actually in "normalized units" from fxpos return?
    )
    # Checking fxpos: returns xpos / samplegridsize. So it IS pixels.

    x_offsets = (lane_idx * x_width_val).astype(np.int16)

    # Generate X: Add lane offsets to the base lane x-positions
    x_final = (x_lane_base + x_offsets).flatten()

    # Generate Y: Alternate between forward and backward lane data based on parity
    is_forward_lane = (np.arange(lanes) % 2 == 0).reshape(-1, 1)
    y_final = np.where(is_forward_lane, y_fwd_lane, y_bwd_lane).flatten()

    # Stack results
    ids = np.vstack((x_final, y_final)).astype(np.int32)

    derived_stats = {
        "lanewidth": lanewidth,
        "facetsinlane": facets_inlane,
        "lanes": lanes,  # Returning lanes is useful for the IO module later
    }

    return ids, derived_stats
