"""Reference 2D grid-sampling geometry model for ``RasterExposure`` and ``plotter``.

Generates pure-NumPy lookup coordinates in native Laserhead coordinates:
``u_scan`` (fast prism sweep + multi-lane stitching) and ``v_step`` (slow stage axis).
"""

import logging
import math
from collections.abc import Iterator
from typing import Any

import numpy as np

from ..config import displacement_at_frac, displacement_kernel

logger = logging.getLogger(__name__)

# Pure-NumPy optical displacement kernel alias
displacement = displacement_kernel


def _pixel_to_frac(
    pixel: float | np.ndarray, params: dict[str, Any]
) -> float | np.ndarray:
    """Map software raster index ``pixel`` in ``[0, bitsinscanline)`` onto ``[start_frac, end_frac]``."""
    n = float(params["bitsinscanline"])
    p = pixel % n
    start_f = float(params["start_frac"])
    end_f = float(params["end_frac"])
    denom = max(1.0, n - 1.0)
    return start_f + (p / denom) * (end_f - start_f)


def fxpos(
    pixel: float | np.ndarray,
    params: dict[str, Any],
    facet_idx: int = 0,
    xstart: float = 0.0,
) -> float | np.ndarray:
    """Return laserhead ``u_scan`` position in sample-grid pixels.

    Args:
        pixel: Pixel index or array along the scanline.
        params: Optical/geometry parameter dictionary.
        facet_idx: Physical prism facet index ``0 .. facets - 1``.
        xstart: Offset along ``u_scan`` in mm.
    """
    dx = float(params[f"f{facet_idx}_scan"])
    frac = _pixel_to_frac(pixel, params)
    xpos = (
        (np.sin(float(params["tiltangle"])) * displacement_at_frac(frac, params))
        + float(xstart)
        + dx
    )
    return xpos / float(params["samplegridsize"])


def fypos(
    pixel: float | np.ndarray,
    params: dict[str, Any],
    facet_idx: int = 0,
    direction: bool = True,
    ystart: float = 0.0,
) -> float | np.ndarray:
    """Return laserhead ``v_step`` position in sample-grid pixels.

    Args:
        pixel: Pixel index or array along the scanline.
        params: Optical/geometry parameter dictionary.
        facet_idx: Physical prism facet index ``0 .. facets - 1``.
        direction: True for forward lane motion, False for backward lane motion.
        ystart: Offset along ``v_step`` in mm.
    """
    dy = float(params[f"f{facet_idx}_orth"])
    frac = _pixel_to_frac(pixel, params)
    base_y = (
        -np.cos(float(params["tiltangle"])) * displacement_at_frac(frac, params)
    ) + dy
    facet_freq = float(params["rotationfrequency"]) * float(params["facets"])
    stage_component = (frac / facet_freq) * float(params["stagespeed"])
    offset = float(ystart)
    if direction:
        ypos = base_y + stage_component + offset
    else:
        ypos = base_y - stage_component + offset
    return ypos / float(params["samplegridsize"])


scan_u_px = fxpos
stage_v_px = fypos


def _plan_grid_geometry(
    params: dict[str, Any],
    lanes: int | None = None,
    facets_inlane: int | None = None,
) -> dict[str, Any]:
    """Plan grid dimensions and precompute per-facet base sweeps."""
    if (lanes is None or facets_inlane is None) and (
        not params["sampleysize"] or not params["samplexsize"]
    ):
        raise ValueError("Sampleysize or samplexsize are set to zero.")

    points_per_sweep = int(params["bitsinscanline"])
    num_physical_facets = int(params["facets"])
    grid_size = float(params["samplegridsize"])
    start_x = float(fxpos(0.0, params, 0, 0.0))
    end_x = float(fxpos(float(points_per_sweep - 1), params, 0, 0.0))

    if start_x < 0 or end_x > 0:
        raise ValueError("Line seems ill positioned: fxpos(0) < 0 or end > 0")

    lanewidth = float((start_x - end_x) * grid_size)
    planned_lanes = (
        math.ceil(float(params["samplexsize"]) / lanewidth)
        if lanes is None
        else int(lanes)
    )
    planned_facets = (
        math.ceil(
            float(params["rotationfrequency"])
            * num_physical_facets
            * (float(params["sampleysize"]) / float(params["stagespeed"]))
        )
        if facets_inlane is None
        else int(facets_inlane)
    )

    xstart_val = float(abs(end_x * grid_size))
    x_width_pixels = float(start_x - end_x)
    stagespeed = float(params["stagespeed"])
    rot_freq = float(params["rotationfrequency"])
    y_shift_per_facet = stagespeed / (num_physical_facets * rot_freq * grid_size)

    # Precompute the F physical facet base sweeps (F, P) in pure NumPy
    pixel_indices = np.arange(points_per_sweep, dtype=np.float64)
    fracs = _pixel_to_frac(pixel_indices, params)
    disp = displacement_at_frac(fracs, params)

    tilt = float(params["tiltangle"])
    sin_tilt = math.sin(tilt)
    cos_tilt = math.cos(tilt)
    stage_comp = (fracs / (rot_freq * num_physical_facets)) * stagespeed

    dx_arr = np.array(
        [float(params[f"f{fi}_scan"]) for fi in range(num_physical_facets)],
        dtype=np.float64,
    )[:, None]
    dy_arr = np.array(
        [float(params[f"f{fi}_orth"]) for fi in range(num_physical_facets)],
        dtype=np.float64,
    )[:, None]

    u_base_mm = (sin_tilt * disp)[None, :] + xstart_val + dx_arr
    base_v_mm = (-cos_tilt * disp)[None, :] + dy_arr
    v_base_fwd_mm = base_v_mm + stage_comp[None, :]
    v_base_bwd_mm = base_v_mm - stage_comp[None, :]

    f_lane_indices = np.arange(planned_facets, dtype=np.int64)
    facet_map = f_lane_indices % num_physical_facets

    u_lane_base_mm = u_base_mm[facet_map]
    v_fwd_mm = (
        v_base_fwd_mm[facet_map]
        + (f_lane_indices * y_shift_per_facet * grid_size)[:, None]
    )
    v_bwd_mm = (
        v_base_bwd_mm[facet_map]
        + ((planned_facets - f_lane_indices) * y_shift_per_facet * grid_size)[:, None]
    )

    params["lanewidth"] = lanewidth
    params["lanes"] = float(planned_lanes)
    params["facetsinlane"] = float(planned_facets)

    return {
        "lanes": planned_lanes,
        "facets_inlane": planned_facets,
        "points_per_sweep": points_per_sweep,
        "grid_size": grid_size,
        "x_width_pixels": x_width_pixels,
        "u_lane_base_mm": u_lane_base_mm,
        "v_fwd_mm": v_fwd_mm,
        "v_bwd_mm": v_bwd_mm,
    }


def calculate_grid(
    params: dict[str, Any],
    lanes: int | None = None,
    facets_inlane: int | None = None,
) -> np.ndarray:
    """Generate the ``(2, N)`` int32 laserhead pixel lookup grid ``[u_px, v_px]``."""
    plan = _plan_grid_geometry(params, lanes=lanes, facets_inlane=facets_inlane)
    n_lanes = plan["lanes"]
    n_facets = plan["facets_inlane"]
    n_pts = plan["points_per_sweep"]
    grid_size = plan["grid_size"]
    x_width_pixels = plan["x_width_pixels"]

    u_lane_base_px = plan["u_lane_base_mm"] / grid_size
    v_fwd_px = (plan["v_fwd_mm"] / grid_size).astype(np.int32)
    v_bwd_px = (plan["v_bwd_mm"] / grid_size).astype(np.int32)

    total_per_lane = n_facets * n_pts
    ids = np.empty((2, n_lanes * total_per_lane), dtype=np.int32)
    u_out = ids[0].reshape(n_lanes, n_facets, n_pts)
    v_out = ids[1].reshape(n_lanes, n_facets, n_pts)

    for lane_idx in range(n_lanes):
        u_out[lane_idx] = (u_lane_base_px + lane_idx * x_width_pixels).astype(np.int32)
        v_out[lane_idx] = v_fwd_px if (lane_idx % 2 == 0) else v_bwd_px

    return ids


def iter_lane_coords_mm(
    params: dict[str, Any],
    lanes: int | None = None,
    facets_inlane: int | None = None,
) -> Iterator[tuple[int, np.ndarray, np.ndarray]]:
    """Yield ``(lane_idx, u_mm, v_mm)`` per lane in physical Laserhead millimeters."""
    plan = _plan_grid_geometry(params, lanes=lanes, facets_inlane=facets_inlane)
    lane_step_mm = plan["x_width_pixels"] * plan["grid_size"]
    u_lane_base_mm = plan["u_lane_base_mm"]
    v_fwd_mm = plan["v_fwd_mm"]
    v_bwd_mm = plan["v_bwd_mm"]

    for lane_idx in range(plan["lanes"]):
        u_mm = u_lane_base_mm + (lane_idx * lane_step_mm)
        v_mm = v_fwd_mm if (lane_idx % 2 == 0) else v_bwd_mm
        yield lane_idx, u_mm, v_mm


class ScannerModel:
    """Pure-NumPy optical scanner geometry wrapper holding system parameters."""

    def __init__(self, params: dict[str, Any]):
        self.params = dict(params)

    def fxpos(self, pixel: float, facet_idx: int = 0, xstart: float = 0.0) -> float:
        return float(fxpos(pixel, self.params, facet_idx, xstart))

    def fypos(
        self, pixel: float, facet_idx: int, direction: bool, ystart: float = 0.0
    ) -> float:
        return float(fypos(pixel, self.params, facet_idx, direction, ystart))

    def calculate_coordinates(
        self,
        lanes: int | None = None,
        facets_inlane: int | None = None,
    ) -> np.ndarray:
        """Generate the ``(2, N)`` int32 lookup grid ``[u_px, v_px]`` in Laserhead space."""
        ids = calculate_grid(self.params, lanes=lanes, facets_inlane=facets_inlane)
        logger.debug(f"The lanewidth is {self.params['lanewidth']:.2f} mm")
        logger.debug(f"The facets in lane are {int(self.params['facetsinlane'])}")
        return ids

    def iter_lane_coords_mm(
        self,
        lanes: int | None = None,
        facets_inlane: int | None = None,
    ) -> Iterator[tuple[int, np.ndarray, np.ndarray]]:
        """Yield ``(lane_idx, u_mm, v_mm)`` per lane in physical Laserhead millimeters."""
        return iter_lane_coords_mm(
            self.params, lanes=lanes, facets_inlane=facets_inlane
        )
