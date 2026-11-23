"""Portable MicroPython + CPython wrapper around the C99 ``vector_core`` slicer.

Uses stdlib ``array`` ('d' float64, 'i' int32) and ``bytearray`` so it packs
exact IEEE-754 doubles and 32-bit ints for ``_vector_core.slice_lane_spi`` on
both ESP32-S3 MicroPython and CPython without requiring NumPy, PIL, or SVG libs.
"""

import math
from array import array

from ..config import Spi
from .core import PrismCurveLite, make_dot_centers_mm

try:
    import _vector_core

    HAS_VECTOR_CORE = hasattr(_vector_core, "slice_lane_spi")
except ImportError:
    try:
        from ..exposure import _vector_core

        HAS_VECTOR_CORE = hasattr(_vector_core, "slice_lane_spi")
    except ImportError:
        _vector_core = None
        HAS_VECTOR_CORE = False

_SENTINEL_TICK = 0xFFFFFF


def circle_polygon(
    cx: float, cy: float, radius: float, n_segments: int = 24
) -> list[tuple[float, float]]:
    """Return closed circle boundary vertices ``[(u, v), ...]`` in mm."""
    n = max(8, int(n_segments))
    step = (2.0 * math.pi) / n
    r = float(radius)
    cx_f = float(cx)
    cy_f = float(cy)
    return [
        (cx_f + r * math.cos(i * step), cy_f + r * math.sin(i * step)) for i in range(n)
    ]


def _normalize_shapes(
    polygons: list,
) -> tuple[
    array,
    array,
    array,
    array,
    array,
    array,
    array,
    array,
    array,
    array,
    array,
    bytearray,
]:
    """Flatten polygon contours into contiguous C-ready buffer arrays."""
    u0_vals: list[float] = []
    v0_vals: list[float] = []
    u1_vals: list[float] = []
    v1_vals: list[float] = []
    du_vals: list[float] = []
    edge_starts: list[int] = []
    edge_counts: list[int] = []
    poly_umin: list[float] = []
    poly_umax: list[float] = []
    poly_vmin: list[float] = []
    poly_vmax: list[float] = []
    fill_rules = bytearray()

    for item in polygons:
        fill_rule = "evenodd"
        if isinstance(item, dict):
            contours = item["contours"]
            fill_rule = str(item.get("fill_rule", "evenodd"))
        elif (
            isinstance(item, tuple)
            and len(item) == 2
            and isinstance(item[1], str)
        ):
            contours, fill_rule = item
        elif hasattr(item, "contours"):
            contours = item.contours
            fill_rule = str(getattr(item, "fill_rule", "evenodd"))
        else:
            contours = [item]

        shape_start = len(u0_vals)
        umin = 0.0
        umax = 0.0
        vmin = 0.0
        vmax = 0.0
        has_pts = False

        for contour in contours:
            pts = [(float(p[0]), float(p[1])) for p in contour]
            n_pts = len(pts)
            if n_pts < 3:
                continue
            for idx in range(n_pts):
                x0, y0 = pts[idx]
                x1, y1 = pts[(idx + 1) % n_pts]
                du = x1 - x0
                dv = y1 - y0
                if abs(du) <= 1e-12 and abs(dv) <= 1e-12:
                    continue
                u0_vals.append(x0)
                v0_vals.append(y0)
                u1_vals.append(x1)
                v1_vals.append(y1)
                du_vals.append(du)
                if not has_pts:
                    umin = umax = min(x0, x1)
                    vmin = vmax = min(y0, y1)
                    umax = max(x0, x1)
                    vmax = max(y0, y1)
                    has_pts = True
                else:
                    if x0 < umin:
                        umin = x0
                    if x1 < umin:
                        umin = x1
                    if x0 > umax:
                        umax = x0
                    if x1 > umax:
                        umax = x1
                    if y0 < vmin:
                        vmin = y0
                    if y1 < vmin:
                        vmin = y1
                    if y0 > vmax:
                        vmax = y0
                    if y1 > vmax:
                        vmax = y1

        shape_count = len(u0_vals) - shape_start
        if shape_count <= 0:
            continue
        edge_starts.append(shape_start)
        edge_counts.append(shape_count)
        poly_umin.append(umin)
        poly_umax.append(umax)
        poly_vmin.append(vmin)
        poly_vmax.append(vmax)
        fill_rules.append(1 if fill_rule == "nonzero" else 0)

    return (
        array("d", u0_vals),
        array("d", v0_vals),
        array("d", u1_vals),
        array("d", v1_vals),
        array("d", du_vals),
        array("i", edge_starts),
        array("i", edge_counts),
        array("d", poly_umin),
        array("d", poly_umax),
        array("d", poly_vmin),
        array("d", poly_vmax),
        fill_rules,
    )


def slice_polygons_to_spi(
    polygons: list,
    params: dict,
    u_size_mm: float,
    v_size_mm: float,
    steps_per_line: float,
    facet_offsets_mm: dict[int, tuple[float, float]] | None = None,
    lane_idx: int = 0,
    facets_inlane: int | None = None,
    invert_hw_scan_dir: bool = True,
    positiveresist: bool = False,
    default_intensity: int = 255,
    n_lut: int = 257,
    xstart_override: float | None = None,
    y_shift_override: float | None = None,
    include_stage_drift: bool = True,
) -> tuple[bytes, int]:
    """Slice laserhead-frame polygons via ``_vector_core`` into raw framed SPI bytes."""
    if not HAS_VECTOR_CORE or _vector_core is None:
        raise RuntimeError("_vector_core C module is not available")

    curve = PrismCurveLite(
        params, invert_hw_scan_dir=invert_hw_scan_dir, n_lut=n_lut
    )
    facets = int(params["facets"])
    rot_freq = float(
        params.get("rotationfrequency", float(params.get("rpm", 3000.0)) / 60.0)
    )
    stagespeed = float(params.get("stagespeed", 0.0))
    if facets_inlane is None:
        if stagespeed <= 0.0:
            raise ValueError("facets_inlane or positive stagespeed is required")
        facets_inlane = math.ceil(rot_freq * facets * (float(v_size_mm) / stagespeed))

    facet_period_s = 1.0 / (rot_freq * facets)
    y_shift_per_facet = (
        float(y_shift_override)
        if y_shift_override is not None
        else stagespeed * facet_period_s
    )
    v_prism_start = -curve.cos_tilt * curve.disp_last
    v_prism_end = -curve.cos_tilt * curve.disp_first
    if include_stage_drift:
        v_stage_start = (curve.end_frac * facet_period_s) * stagespeed
        v_stage_end = (curve.start_frac * facet_period_s) * stagespeed
    else:
        v_stage_start = 0.0
        v_stage_end = 0.0

    slope_den = curve.raw_u_first - curve.raw_u_last
    xstart_val = curve.xstart_val if xstart_override is None else float(xstart_override)

    offsets = facet_offsets_mm or {}
    dx_list = []
    dy_list = []
    for f_idx in range(facets):
        if f_idx in offsets:
            dx_f, dy_f = offsets[f_idx]
        else:
            dx_f = params.get(f"f{f_idx}_scan", 0.0)
            dy_f = params.get(f"f{f_idx}_orth", 0.0)
        dx_list.append(float(dx_f))
        dy_list.append(float(dy_f))

    (
        u0_arr,
        v0_arr,
        u1_arr,
        v1_arr,
        du_arr,
        edge_starts,
        edge_counts,
        poly_umin,
        poly_umax,
        poly_vmin,
        poly_vmax,
        fill_rules,
    ) = _normalize_shapes(polygons)

    u_samples = array("d", curve._u_samples)
    tick_samples = array("d", curve._t_samples)
    dx_facet_arr = array("d", dx_list)
    dy_facet_arr = array("d", dy_list)

    return _vector_core.slice_lane_spi(
        u0_arr,
        v0_arr,
        u1_arr,
        v1_arr,
        du_arr,
        edge_starts,
        edge_counts,
        poly_umin,
        poly_umax,
        poly_vmin,
        poly_vmax,
        fill_rules,
        u_samples,
        tick_samples,
        dx_facet_arr,
        dy_facet_arr,
        1 if curve.invert_hw_scan_dir else 0,
        int(curve.total_ticks),
        int(lane_idx),
        int(facets_inlane),
        float(steps_per_line),
        float(u_size_mm),
        float(v_size_mm),
        float(curve.lanewidth),
        float(curve.raw_u_last),
        float(xstart_val),
        float(slope_den),
        float(y_shift_per_facet),
        float(v_prism_start),
        float(v_prism_end),
        float(v_stage_start),
        float(v_stage_end),
        bool(positiveresist),
        int(default_intensity),
        int(Spi.Instructions.scanline),
        int(Spi.Commands.write),
    )


def decode_spi_scanlines(
    raw_spi: bytes,
    num_facets: int = 4,
    skip_empty: bool = True,
) -> list[tuple[int, int, list[tuple[int, int]]]]:
    """Decode raw 9-byte SPI packets from ``slice_lane_spi`` into row events."""
    pkt_len = Spi.command_bytes + Spi.word_bytes
    write_cmd = Spi.Commands.write
    scanline_cmd = Spi.Instructions.scanline
    n_bytes = len(raw_spi)
    idx = 0
    row_idx = 0
    rows: list[tuple[int, int, list[tuple[int, int]]]] = []

    while idx + pkt_len <= n_bytes:
        if raw_spi[idx] != write_cmd or raw_spi[idx + pkt_len - 1] != scanline_cmd:
            idx += pkt_len
            continue
        payload = int.from_bytes(raw_spi[idx + 1 : idx + pkt_len - 1], "big")
        event_words = (payload >> 1) & 0xFFFF
        idx += pkt_len

        events: list[tuple[int, int]] = []
        for _ in range(event_words):
            if idx + pkt_len > n_bytes:
                break
            word64 = int.from_bytes(raw_spi[idx + 1 : idx + pkt_len], "big")
            idx += pkt_len

            ev0 = word64 & 0xFFFFFFFF
            tick0 = (ev0 >> 8) & 0xFFFFFF
            if tick0 != _SENTINEL_TICK:
                events.append((tick0, ev0 & 0xFF))

            ev1 = (word64 >> 32) & 0xFFFFFFFF
            tick1 = (ev1 >> 8) & 0xFFFFFF
            if tick1 != _SENTINEL_TICK:
                events.append((tick1, ev1 & 0xFF))

        if events or not skip_empty:
            rows.append((row_idx, row_idx % num_facets, events))
        row_idx += 1

    return rows


def slice_dot_grid_vector_core(
    u_min_mm: float,
    u_max_mm: float,
    params: dict,
    num_dots_x: int = 6,
    num_dots_y: int = 1,
    dot_diameter_mm: float = 0.08,
    exposures: int = 2,
    steps_mm: float = 100.0,
    facet_offsets_mm: dict[int, tuple[float, float]] | None = None,
    invert_hw_scan_dir: bool = True,
    intensity: int = 255,
    circle_segments: int = 24,
) -> dict:
    """Slice a 2D circular dot grid through the C99 ``_vector_core`` kernel.

    Produces the exact same dictionary layout as :func:`slice_dot_grid_events`,
    aligning stationary camera coordinates (no intra-scan stage drift) cleanly.
    """
    curve = PrismCurveLite(params, invert_hw_scan_dir=invert_hw_scan_dir)
    num_facets = int(params["facets"])
    y_step_mm = (1.0 / float(exposures)) / float(steps_mm)
    radius = 0.5 * float(dot_diameter_mm)
    margin_mm = max(0.25, float(dot_diameter_mm))

    xs = make_dot_centers_mm(
        u_min_mm, u_max_mm, num_dots=num_dots_x, margin_mm=margin_mm
    )
    y_offset = 0.15
    spacing_y = max(float(dot_diameter_mm) * 2.5, 0.20)
    dots = [
        (cx, y_offset + j * spacing_y)
        for j in range(max(1, int(num_dots_y)))
        for cx in xs
    ]

    max_cy = max(cy for _cx, cy in dots)
    total_rows = max(1, math.ceil((max_cy + radius + 0.05) / y_step_mm))
    polygons = [
        circle_polygon(cx, cy, radius, n_segments=circle_segments) for cx, cy in dots
    ]

    raw_spi, _active_lines = slice_polygons_to_spi(
        polygons=polygons,
        params=params,
        u_size_mm=curve.lanewidth,
        v_size_mm=(total_rows + 1) * y_step_mm,
        steps_per_line=1.0 / float(exposures),
        facet_offsets_mm=facet_offsets_mm,
        lane_idx=0,
        facets_inlane=total_rows,
        invert_hw_scan_dir=invert_hw_scan_dir,
        positiveresist=False,
        default_intensity=intensity,
        xstart_override=-curve.raw_u_last,
        y_shift_override=y_step_mm,
        include_stage_drift=False,
    )
    rows_out = decode_spi_scanlines(raw_spi, num_facets=num_facets, skip_empty=True)

    return {
        "rows": rows_out,
        "total_rows": total_rows,
        "y_step_mm": y_step_mm,
        "num_facets": num_facets,
        "dot_centers": dots,
        "slicer": "vector_core",
    }
