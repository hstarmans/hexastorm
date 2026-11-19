import logging
from typing import Any

import cv2
import numpy as np

from hexastorm.config import PlatformConfig

from . import geometry

logger = logging.getLogger(__name__)


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


def plot_pattern(
    ptrn_data: dict,
    cfg: PlatformConfig | None = None,
    correction: bool | dict = False,
    exposures: int = 1,
    filename: str = "plot",
    color_lanes: bool = True,
) -> np.ndarray:
    """Visualize an event-driven ``.pat``/``.bin`` pattern dictionary onto an RGB PNG canvas."""
    if not isinstance(ptrn_data, dict) or "data" not in ptrn_data:
        raise ValueError("Could not find 'data' in pattern input structure")

    ptrn_bits = ptrn_data["data"]
    if hasattr(ptrn_bits, "values"):
        ptrn_bits = ptrn_bits.values[0]
    metadata = ptrn_data.get("metadata") or {}

    if cfg is None:
        cfg = PlatformConfig(test=False)

    raw_params = cfg.get_optical_params(correction=correction, exposures=exposures)
    local_params = geometry.ScannerModel.to_numba_dict(raw_params)

    start_x = geometry._jit_fxpos(0.0, local_params, 0, 0.0)
    end_x = geometry._jit_fxpos(
        local_params["bitsinscanline"] - 1, local_params, 0, 0.0
    )
    calc_lanewidth = (start_x - end_x) * local_params["samplegridsize"]

    local_params["samplexsize"] = float(metadata["lanes"] * calc_lanewidth)
    term = local_params["rotationfrequency"] * local_params["facets"]
    local_params["sampleysize"] = float(
        metadata["facetsinlane"] * local_params["stagespeed"] / term
    )
    local_params["lanes"] = float(metadata["lanes"])
    local_params["facetsinlane"] = float(metadata["facetsinlane"])

    ids = geometry._jit_calculate_grid(local_params)

    factor = int(local_params["downsamplefactor"])
    if factor > 1:
        ids = np.repeat(ids, factor, axis=1)

    min_len = min(ids.shape[1], len(ptrn_bits))
    x_plot = ids[0, :min_len]
    y_plot = ids[1, :min_len]
    bits_plot = ptrn_bits[:min_len]

    x_min, x_max = x_plot.min(), x_plot.max()
    y_min, y_max = y_plot.min(), y_plot.max()
    x_plot = x_plot - x_min
    y_plot = y_plot - y_min

    width = x_max - x_min + 1
    height = y_max - y_min + 1
    logger.debug(f"Plotting Image: {width}x{height} pixels")

    canvas = np.zeros((width, height), dtype=np.uint8)
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
                canvas[x_plot[global_indices], y_plot[global_indices]] = val
    else:
        active_mask = bits_plot == 1
        canvas[x_plot[active_mask], y_plot[active_mask]] = val_all

    canvas_dilated = laser_compensation(canvas, local_params, erode=False)

    rgb_canvas = np.zeros((width, height, 3), dtype=np.uint8)
    if color_lanes:
        rgb_canvas[canvas_dilated == val_even] = (255, 200, 100)
        rgb_canvas[canvas_dilated == val_odd] = (255, 255, 255)
    else:
        rgb_canvas[canvas_dilated > 0] = (255, 255, 255)

    rotated_canvas = cv2.rotate(rgb_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)

    debug_folder = cfg.paths["debug"]
    debug_folder.mkdir(parents=True, exist_ok=True)
    save_path = debug_folder / f"{filename}.png"
    cv2.imwrite(str(save_path), rotated_canvas)
    logger.info(f"Plot saved to {save_path}")
    return rotated_canvas
