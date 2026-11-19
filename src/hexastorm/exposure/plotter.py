import logging
from typing import Any

import cv2
import numpy as np

from hexastorm.config import PlatformConfig

from . import geometry
from .coordinates import CoordinateTransform, MachineFrame

logger = logging.getLogger(__name__)

VALID_VIEWS = ("laserhead", "substrate", "machine")


def laser_compensation(
    layerarr: np.ndarray, params: Any, erode: bool = False
) -> np.ndarray:
    """Apply elliptical laser-spot morphology (erosion or dilation) using OpenCV."""
    pixel_radius = params["laser_radius_mm"] / params["samplegridsize"]
    kernel_size = round(pixel_radius * 2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    logger.debug(
        f"Fast OpenCV bias: {params['laser_radius_mm'] * 1000:.1f}um "
        f"({pixel_radius:.2f} px radius, {kernel_size}px kernel)"
    )

    if erode:
        result = cv2.erode(layerarr, kernel, iterations=1)
    else:
        result = cv2.dilate(layerarr, kernel, iterations=1)

    return result.astype(np.uint8)


def _map_to_display_coords(
    u_px: np.ndarray,
    v_px: np.ndarray,
    grid_size: float,
    view: str,
    mach: MachineFrame,
    coord: CoordinateTransform | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Map Laserhead pixel grid ``(u_px, v_px)`` into right-handed display pixels."""
    if view == "laserhead" or (
        view == "substrate" and (coord is None or coord.is_identity_lh_to_sub)
    ):
        return u_px, v_px

    if view == "machine":
        if mach.step_axis == "x":
            u_max = int(u_px.max()) if u_px.size else 0
            return v_px, (u_max - u_px)
        return u_px, v_px

    assert coord is not None
    x_sub_mm, y_sub_mm = coord.laserhead_to_substrate_xy(
        u_px * grid_size, v_px * grid_size
    )
    return (
        np.rint(x_sub_mm / grid_size).astype(np.int32),
        np.rint(y_sub_mm / grid_size).astype(np.int32),
    )


def plot_pattern(
    ptrn_data: dict,
    cfg: PlatformConfig | None = None,
    correction: bool | dict = False,
    exposures: int = 1,
    filename: str = "plot",
    color_lanes: bool = True,
    view: str = "laserhead",
    machine: MachineFrame | None = None,
    coord: CoordinateTransform | None = None,
) -> np.ndarray:
    """Visualize an event-driven ``.pat``/``.bin`` pattern dictionary onto an RGB PNG canvas."""
    if view not in VALID_VIEWS:
        raise ValueError(f"Unsupported plot view {view!r}; choose from {VALID_VIEWS}")
    if not isinstance(ptrn_data, dict) or "data" not in ptrn_data:
        raise ValueError("Could not find 'data' in pattern input structure")

    ptrn_bits = ptrn_data["data"]
    if hasattr(ptrn_bits, "values"):
        ptrn_bits = ptrn_bits.values[0]
    metadata = ptrn_data.get("metadata") or {}

    if cfg is None:
        cfg = PlatformConfig(test=False)

    mach = machine if machine is not None else MachineFrame.from_config(cfg)
    raw_params = cfg.get_optical_params(correction=correction, exposures=exposures)
    local_params = dict(raw_params)

    lanes = int(metadata["lanes"])
    facets_inlane = int(metadata["facetsinlane"])

    start_x = geometry.fxpos(0.0, local_params, 0, 0.0)
    end_x = geometry.fxpos(local_params["bitsinscanline"] - 1, local_params, 0, 0.0)
    calc_lanewidth = (start_x - end_x) * local_params["samplegridsize"]

    local_params["samplexsize"] = float(lanes * calc_lanewidth)
    term = local_params["rotationfrequency"] * local_params["facets"]
    local_params["sampleysize"] = float(
        facets_inlane * local_params["stagespeed"] / term
    )
    local_params["lanes"] = float(lanes)
    local_params["facetsinlane"] = float(facets_inlane)

    ids = geometry.calculate_grid(
        local_params, lanes=lanes, facets_inlane=facets_inlane
    )

    grid_size = float(local_params["samplegridsize"])
    x_disp, y_disp = _map_to_display_coords(
        ids[0],
        ids[1],
        grid_size=grid_size,
        view=view,
        mach=mach,
        coord=coord,
    )

    factor = int(local_params["downsamplefactor"])
    if factor > 1:
        x_disp = np.repeat(x_disp, factor)
        y_disp = np.repeat(y_disp, factor)

    min_len = min(len(x_disp), len(ptrn_bits))
    x_plot = x_disp[:min_len]
    y_plot = y_disp[:min_len]
    bits_plot = ptrn_bits[:min_len]

    x_plot = x_plot - x_plot.min()
    y_plot = y_plot - y_plot.min()

    width = int(x_plot.max()) + 1
    height = int(y_plot.max()) + 1
    row_plot = (height - 1) - y_plot
    logger.debug(f"Plotting Image: {width}x{height} pixels (view={view})")

    canvas = np.zeros((height, width), dtype=np.uint8)
    val_even = 100
    val_odd = 200
    val_all = 255

    if color_lanes:
        num_lanes = int(local_params.get("lanes", 1))
        samples_per_lane = len(x_plot) // num_lanes
        for i in range(num_lanes):
            start = i * samples_per_lane
            end = start + samples_per_lane if i < num_lanes - 1 else len(x_plot)
            lbits = bits_plot[start:end]
            active_indices = np.where(lbits == 1)[0]
            if active_indices.size > 0:
                global_indices = start + active_indices
                val = val_even if (i % 2 == 0) else val_odd
                canvas[row_plot[global_indices], x_plot[global_indices]] = val
    else:
        active_mask = bits_plot == 1
        canvas[row_plot[active_mask], x_plot[active_mask]] = val_all

    canvas_dilated = laser_compensation(canvas, local_params, erode=False)

    rgb_canvas = np.zeros((height, width, 3), dtype=np.uint8)
    if color_lanes:
        rgb_canvas[canvas_dilated == val_even] = (255, 200, 100)
        rgb_canvas[canvas_dilated == val_odd] = (255, 255, 255)
    else:
        rgb_canvas[canvas_dilated > 0] = (255, 255, 255)

    debug_folder = cfg.paths["debug"]
    debug_folder.mkdir(parents=True, exist_ok=True)
    save_path = debug_folder / f"{filename}.png"
    cv2.imwrite(str(save_path), rgb_canvas)
    logger.info(f"Plot saved to {save_path}")
    return rgb_canvas
