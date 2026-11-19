"""Direct Vector-to-Timestamp slicer and SVG polygon parser (Fase 3).

Converts SVG vector geometries or raw 2D polygon contours directly into variable-length
event-driven ``[tick, intensity]`` FPGA scanline files (``.pat`` / ``.bin``) without
intermediate bitmap rasterization.

Coordinate Frames:
    1. Substrate frame ``(x_sub, y_sub)`` [mm]: bottom-left origin ``(0, 0)``,
       right = ``+x``, up = ``+y``.
    2. Machine frame ``(x_mach, y_mach)`` [mm]: physical stage coordinates, mapped
       from Substrate via :class:`SubstratePlacement`.
    3. Laserhead frame ``(u_scan, v_step)`` [mm]: optical scanhead coordinates,
       mapped from Machine via :class:`MachineFrame` and :class:`LaserheadMount`.
       ``u_scan`` is the fast prism-scan axis (divided into lanes of width
       ``lanewidth``), and ``v_step`` is the slow stage-stepping axis.
    4. FPGA Clock Ticks ``t``: per scanline ``[0, scanline_length * laser_ticks]``,
       computed directly from intersection coordinates ``u_hit`` via the 1D inverse
       Snellius prism curve (:class:`PrismCurve`).

The pure-NumPy bitmap pipeline (:mod:`hexastorm.exposure.raster` and
:mod:`hexastorm.exposure.geometry`) serves as the reference implementation for
verification and pure raster image fallback.
"""

from __future__ import annotations

import logging
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..config import PlatformConfig, Spi, displacement_kernel
from . import io, plotter
from .coordinates import (
    CoordinateTransform,
    LaserheadMount,
    MachineFrame,
    SubstratePlacement,
)

try:
    from . import _vector_core

    _HAS_C_CORE = _vector_core is not None
except ImportError:
    _vector_core = None
    _HAS_C_CORE = False

logger = logging.getLogger(__name__)

_SVG_CMD_RE = re.compile(
    r"([MmLlHhVvCcSsQqTtAaZz])|([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)"
)
_LENGTH_RE = re.compile(
    r"^\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s*([a-zA-Z%]*)\s*$"
)
_TRANSFORM_RE = re.compile(r"([a-zA-Z]+)\s*\(([^)]*)\)")
_NUM_RE = re.compile(r"[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")

_UNIT_TO_MM = {
    "": 25.4 / 96.0,
    "px": 25.4 / 96.0,
    "pt": 25.4 / 72.0,
    "pc": 25.4 / 6.0,
    "mm": 1.0,
    "cm": 10.0,
    "in": 25.4,
}


@dataclass
class VectorShape:
    """One closed SVG/vector graphic element consisting of one or more contours.

    Grouping contours per element ensures overlapping separate elements take a
    boolean union while interior holes inside a single ``<path>`` obey its
    ``fill_rule`` (``"evenodd"`` or ``"nonzero"``).
    """

    contours: list[np.ndarray]
    fill_rule: str = "evenodd"
    intensity: int | None = None

    def v_bounds(self) -> tuple[float, float]:
        """Return ``(v_min, v_max)`` across all contours of this shape."""
        vmin = min(float(c[:, 1].min()) for c in self.contours if len(c) >= 3)
        vmax = max(float(c[:, 1].max()) for c in self.contours if len(c) >= 3)
        return vmin, vmax


class PrismCurve:
    """1D forward and inverse mapping between lane-local ``u`` [mm] and FPGA clock ticks.

    At continuous hardware tick ``t`` in ``[0, bitsinscanline * laser_ticks]``:
    - When ``invert_hw_scan_dir=True`` (Hexastorm standard), tick ``0`` aligns with
      the outer edge ``p = N - 0.5`` of hardware pixel ``0`` (legacy pixel ``N - 1``)
      and tick ``N * laser_ticks`` aligns with ``p = -0.5`` of hardware pixel ``N - 1``.
    - Pixel center ``p`` (``0 <= p <= N - 1``) corresponds to hardware tick
      ``((N - 1 - p) + 0.5) * laser_ticks``.
    """

    def __init__(self, params: dict[str, Any], invert_hw_scan_dir: bool = True):
        self.bits_in_scanline = int(params["bitsinscanline"])
        self.laser_ticks = int(params["laser_ticks"])
        self.total_ticks = self.bits_in_scanline * self.laser_ticks
        self.start_pixel = float(params["startpixel"])
        self.tiltangle = float(params["tiltangle"])
        self.sin_tilt = math.sin(self.tiltangle)
        self.cos_tilt = math.cos(self.tiltangle)
        self.invert_hw_scan_dir = invert_hw_scan_dir
        self.f0_scan = float(params.get("f0_scan", 0.0))

        # Sample pixel centers p = [N-1, N-2, ..., 0] extended by +-0.5 px at ends
        # so tick=0..total_ticks covers the full physical aperture seamlessly.
        n = self.bits_in_scanline
        self._p_samples = np.concatenate(
            ([n - 0.5], np.arange(n - 1, -1, -1, dtype=np.float64), [-0.5])
        )
        disp_samples = np.array(
            [
                displacement_kernel(self.start_pixel + float(p), params)
                for p in self._p_samples
            ],
            dtype=np.float64,
        )
        raw_u = self.sin_tilt * disp_samples
        disp_first_center = displacement_kernel(self.start_pixel, params)
        disp_last_center = displacement_kernel(self.start_pixel + (n - 1), params)
        self.raw_u_first = self.sin_tilt * disp_first_center
        self.raw_u_last = self.sin_tilt * disp_last_center
        self.disp_first = disp_first_center
        self.disp_last = disp_last_center

        self.xstart_val = abs(self.raw_u_last + self.f0_scan)
        self.lanewidth = abs(self.raw_u_first - self.raw_u_last)

        # u_base(p) = raw_u(p) + xstart_val - f0_scan -> 0.0 at p=N-1, lanewidth at p=0
        self._u_base_samples = raw_u - self.raw_u_last
        # Corresponding hardware ticks if invert_hw_scan_dir is True:
        # p = N - 0.5 -> tick = 0, p = N - 1 -> tick = 0.5*laser_ticks, ..., p = -0.5 -> total_ticks
        self._tick_samples = ((n - 0.5) - self._p_samples) * float(self.laser_ticks)

        # Ensure u is strictly ascending for np.interp
        if self._u_base_samples[0] > self._u_base_samples[-1]:
            self._u_base_samples = self._u_base_samples[::-1].copy()
            self._tick_samples = self._tick_samples[::-1].copy()

        self.u_base_min = float(self._u_base_samples[0])
        self.u_base_max = float(self._u_base_samples[-1])

    def u_to_tick(self, u_base_mm: float | np.ndarray) -> int | np.ndarray:
        """Convert lane-local ``u`` [mm] (with facet offset already removed) to FPGA tick."""
        t = np.interp(
            u_base_mm,
            self._u_base_samples,
            self._tick_samples,
            left=float(self._tick_samples[0]),
            right=float(self._tick_samples[-1]),
        )
        if not self.invert_hw_scan_dir:
            t = float(self.total_ticks) - t
        ticks = np.clip(np.rint(t), 0, self.total_ticks).astype(np.int64)
        return int(ticks) if np.ndim(u_base_mm) == 0 else ticks

    def tick_to_u(self, tick: float | np.ndarray) -> float | np.ndarray:
        """Inverse helper: map FPGA clock tick back to lane-local ``u_base`` [mm]."""
        t_arr = np.asarray(tick, dtype=np.float64)
        if not self.invert_hw_scan_dir:
            t_arr = float(self.total_ticks) - t_arr
        # _tick_samples might be descending if _u_base_samples was reversed above
        if self._tick_samples[0] <= self._tick_samples[-1]:
            ts = self._tick_samples
            us = self._u_base_samples
        else:
            ts = self._tick_samples[::-1]
            us = self._u_base_samples[::-1]
        u = np.interp(t_arr, ts, us, left=float(us[0]), right=float(us[-1]))
        return float(u) if np.ndim(tick) == 0 else u


# ---------------------------------------------------------------------------
# SVG Length, Transform & Path Flattening Helpers
# ---------------------------------------------------------------------------


def _parse_length_to_user(value: str | None, default_px: float = 0.0) -> float:
    """Parse an SVG length string into raw SVG user/CSS pixel units when no root scale exists."""
    if not value:
        return default_px
    m = _LENGTH_RE.match(value)
    if not m:
        return default_px
    num = float(m.group(1))
    unit = m.group(2).lower()
    if unit == "mm":
        return num * (96.0 / 25.4)
    if unit == "cm":
        return num * (96.0 / 2.54)
    if unit == "in":
        return num * 96.0
    if unit == "pt":
        return num * (96.0 / 72.0)
    if unit == "pc":
        return num * (96.0 / 6.0)
    return num


def _parse_length_to_mm(
    value: str | None, user_to_mm: float = _UNIT_TO_MM[""], default: float = 0.0
) -> float:
    """Parse an SVG attribute length (`width`, `height`, `stroke-width`) into mm."""
    if not value:
        return default
    m = _LENGTH_RE.match(value)
    if not m:
        return default
    num = float(m.group(1))
    unit = m.group(2).lower()
    if unit in _UNIT_TO_MM and unit != "":
        return num * _UNIT_TO_MM[unit]
    return num * user_to_mm


def _parse_transform(transform_str: str | None) -> np.ndarray:
    """Parse an SVG ``transform`` attribute into a 3x3 affine matrix."""
    mat = np.eye(3, dtype=np.float64)
    if not transform_str:
        return mat
    for fn, raw_args in _TRANSFORM_RE.findall(transform_str):
        args = [float(x) for x in _NUM_RE.findall(raw_args)]
        fn = fn.lower()
        step = np.eye(3, dtype=np.float64)
        if fn == "translate" and len(args) >= 1:
            tx = args[0]
            ty = args[1] if len(args) >= 2 else 0.0
            step[0, 2] = tx
            step[1, 2] = ty
        elif fn == "scale" and len(args) >= 1:
            sx = args[0]
            sy = args[1] if len(args) >= 2 else sx
            step[0, 0] = sx
            step[1, 1] = sy
        elif fn == "rotate" and len(args) >= 1:
            rad = math.radians(args[0])
            c, s = math.cos(rad), math.sin(rad)
            if len(args) >= 3:
                cx, cy = args[1], args[2]
                t1 = np.array([[1, 0, cx], [0, 1, cy], [0, 0, 1]], dtype=np.float64)
                r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
                t2 = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]], dtype=np.float64)
                step = t1 @ r @ t2
            else:
                step = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
        elif fn == "skewx" and len(args) >= 1:
            step[0, 1] = math.tan(math.radians(args[0]))
        elif fn == "skewy" and len(args) >= 1:
            step[1, 0] = math.tan(math.radians(args[0]))
        elif fn == "matrix" and len(args) >= 6:
            a, b, c, d, e, f = args[:6]
            step = np.array([[a, c, e], [b, d, f], [0, 0, 1]], dtype=np.float64)
        mat = mat @ step
    return mat


def _apply_matrix(pts: np.ndarray, mat: np.ndarray) -> np.ndarray:
    """Apply 3x3 affine matrix ``mat`` to ``(K, 2)`` points."""
    ones = np.ones((len(pts), 1), dtype=np.float64)
    hom = np.hstack((pts, ones))
    out = hom @ mat.T
    return out[:, :2]


def _parse_style(elem: ET.Element, parent_style: dict[str, str]) -> dict[str, str]:
    """Merge inherited SVG presentation attributes and inline CSS ``style``."""
    merged = dict(parent_style)
    for key in (
        "fill",
        "fill-rule",
        "fill-opacity",
        "stroke",
        "stroke-width",
        "stroke-linecap",
        "stroke-linejoin",
        "stroke-opacity",
        "display",
        "visibility",
    ):
        val = elem.attrib.get(key)
        if val is not None:
            merged[key] = val.strip()
    style_str = elem.attrib.get("style", "")
    if style_str:
        for decl in style_str.split(";"):
            if ":" in decl:
                k, v = decl.split(":", 1)
                merged[k.strip().lower()] = v.strip()
    return merged


def _is_none_color(color: str | None) -> bool:
    if color is None:
        return True
    c = color.strip().lower()
    return c in ("", "none", "transparent")


def _is_white_color(color: str | None) -> bool:
    if not color:
        return False
    c = color.strip().lower().replace(" ", "")
    return c in ("#fff", "#ffffff", "white", "rgb(255,255,255)", "rgb(100%,100%,100%)")


def _flatten_cubic(
    p0: complex, p1: complex, p2: complex, p3: complex, tol: float
) -> list[complex]:
    """Recursively flatten a cubic Bezier within chord tolerance ``tol``."""
    # Max distance of control points p1, p2 from chord p0->p3
    chord = p3 - p0
    chord_len = abs(chord)
    if chord_len < 1e-12:
        d1 = abs(p1 - p0)
        d2 = abs(p2 - p0)
    else:
        u = chord / chord_len
        d1 = abs(((p1 - p0) * u.conjugate()).imag)
        d2 = abs(((p2 - p0) * u.conjugate()).imag)
    if max(d1, d2) <= tol:
        return [p3]
    p01 = 0.5 * (p0 + p1)
    p12 = 0.5 * (p1 + p2)
    p23 = 0.5 * (p2 + p3)
    p012 = 0.5 * (p01 + p12)
    p123 = 0.5 * (p12 + p23)
    p0123 = 0.5 * (p012 + p123)
    return _flatten_cubic(p0, p01, p012, p0123, tol) + _flatten_cubic(
        p0123, p123, p23, p3, tol
    )


def _flatten_quad(p0: complex, p1: complex, p2: complex, tol: float) -> list[complex]:
    """Elevate quadratic Bezier to cubic and flatten within tolerance ``tol``."""
    c1 = p0 + (2.0 / 3.0) * (p1 - p0)
    c2 = p2 + (2.0 / 3.0) * (p1 - p2)
    return _flatten_cubic(p0, c1, c2, p2, tol)


def _flatten_arc(
    p0: complex,
    rx: float,
    ry: float,
    phi_deg: float,
    large_arc: bool,
    sweep: bool,
    p1: complex,
    tol: float,
) -> list[complex]:
    """Convert an SVG elliptical arc ``A`` command into a polyline within ``tol``."""
    if p0 == p1:
        return []
    rx, ry = abs(rx), abs(ry)
    if rx < 1e-12 or ry < 1e-12:
        return [p1]

    phi = math.radians(phi_deg % 360.0)
    cos_p, sin_p = math.cos(phi), math.sin(phi)
    dx2 = (p0.real - p1.real) * 0.5
    dy2 = (p0.imag - p1.imag) * 0.5
    x1p = cos_p * dx2 + sin_p * dy2
    y1p = -sin_p * dx2 + cos_p * dy2

    # Ensure radii are large enough
    lam = (x1p**2) / (rx**2) + (y1p**2) / (ry**2)
    if lam > 1.0:
        s_lam = math.sqrt(lam)
        rx *= s_lam
        ry *= s_lam

    rx2, ry2 = rx**2, ry**2
    x1p2, y1p2 = x1p**2, y1p**2
    num = max(0.0, rx2 * ry2 - rx2 * y1p2 - ry2 * x1p2)
    den = rx2 * y1p2 + ry2 * x1p2
    factor = math.sqrt(num / den) if den > 1e-18 else 0.0
    if large_arc == sweep:
        factor = -factor

    cxp = factor * (rx * y1p) / ry
    cyp = factor * (-ry * x1p) / rx
    cx = cos_p * cxp - sin_p * cyp + (p0.real + p1.real) * 0.5
    cy = sin_p * cxp + cos_p * cyp + (p0.imag + p1.imag) * 0.5

    def _vec_angle(ux: float, uy: float, vx: float, vy: float) -> float:
        dot = ux * vx + uy * vy
        mag = math.hypot(ux, uy) * math.hypot(vx, vy)
        ang = math.acos(max(-1.0, min(1.0, dot / mag))) if mag > 1e-18 else 0.0
        if ux * vy - uy * vx < 0:
            ang = -ang
        return ang

    ux, uy = (x1p - cxp) / rx, (y1p - cyp) / ry
    vx, vy = (-x1p - cxp) / rx, (-y1p - cyp) / ry
    theta1 = _vec_angle(1.0, 0.0, ux, uy)
    dtheta = _vec_angle(ux, uy, vx, vy)
    if not sweep and dtheta > 0:
        dtheta -= 2.0 * math.pi
    elif sweep and dtheta < 0:
        dtheta += 2.0 * math.pi

    r_max = max(rx, ry)
    max_step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - min(0.5, tol / r_max))))
    n_segs = max(2, math.ceil(abs(dtheta) / max(0.02, max_step)))
    pts: list[complex] = []
    for k in range(1, n_segs):
        t = theta1 + dtheta * (k / n_segs)
        px = rx * math.cos(t)
        py = ry * math.sin(t)
        x = cos_p * px - sin_p * py + cx
        y = sin_p * px + cos_p * py + cy
        pts.append(complex(x, y))
    pts.append(p1)
    return pts


def _parse_svg_path_d(d_str: str, tol_user: float) -> list[tuple[np.ndarray, bool]]:
    """Parse an SVG ``path`` ``d`` string into ``[(points_Nx2, is_closed), ...]``."""
    tokens = []
    for cmd_tok, num_tok in _SVG_CMD_RE.findall(d_str):
        if cmd_tok:
            tokens.append(cmd_tok)
        else:
            tokens.append(float(num_tok))

    subpaths: list[tuple[np.ndarray, bool]] = []
    cur_pts: list[complex] = []
    cur = 0j
    start = 0j
    last_ctrl_c: complex | None = None
    last_ctrl_q: complex | None = None
    cmd = ""
    i = 0
    n_tok = len(tokens)

    def _close_subpath(closed: bool) -> None:
        if len(cur_pts) >= 2:
            arr = np.array([(p.real, p.imag) for p in cur_pts], dtype=np.float64)
            subpaths.append((arr, closed))

    while i < n_tok:
        tok = tokens[i]
        if isinstance(tok, str):
            cmd = tok
            i += 1
            if cmd in ("Z", "z"):
                _close_subpath(closed=True)
                cur = start
                cur_pts = []
                last_ctrl_c = None
                last_ctrl_q = None
            continue

        if cmd in ("M", "m"):
            if cur_pts:
                _close_subpath(closed=False)
                cur_pts = []
            x, y = tokens[i], tokens[i + 1]
            i += 2
            cur = complex(x, y) if cmd == "M" else cur + complex(x, y)
            start = cur
            cur_pts = [cur]
            last_ctrl_c = None
            last_ctrl_q = None
            cmd = "L" if cmd == "M" else "l"
        elif cmd in ("L", "l"):
            x, y = tokens[i], tokens[i + 1]
            i += 2
            nxt = complex(x, y) if cmd == "L" else cur + complex(x, y)
            cur_pts.append(nxt)
            cur = nxt
            last_ctrl_c = None
            last_ctrl_q = None
        elif cmd in ("H", "h"):
            x = tokens[i]
            i += 1
            nxt = (
                complex(x, cur.imag) if cmd == "H" else complex(cur.real + x, cur.imag)
            )
            cur_pts.append(nxt)
            cur = nxt
            last_ctrl_c = None
            last_ctrl_q = None
        elif cmd in ("V", "v"):
            y = tokens[i]
            i += 1
            nxt = (
                complex(cur.real, y) if cmd == "V" else complex(cur.real, cur.imag + y)
            )
            cur_pts.append(nxt)
            cur = nxt
            last_ctrl_c = None
            last_ctrl_q = None
        elif cmd in ("C", "c"):
            x1, y1, x2, y2, x, y = tokens[i : i + 6]
            i += 6
            base = 0j if cmd == "C" else cur
            c1 = base + complex(x1, y1)
            c2 = base + complex(x2, y2)
            p3 = base + complex(x, y)
            cur_pts.extend(_flatten_cubic(cur, c1, c2, p3, tol_user))
            cur = p3
            last_ctrl_c = c2
            last_ctrl_q = None
        elif cmd in ("S", "s"):
            x2, y2, x, y = tokens[i : i + 4]
            i += 4
            base = 0j if cmd == "S" else cur
            c1 = (2 * cur - last_ctrl_c) if last_ctrl_c is not None else cur
            c2 = base + complex(x2, y2)
            p3 = base + complex(x, y)
            cur_pts.extend(_flatten_cubic(cur, c1, c2, p3, tol_user))
            cur = p3
            last_ctrl_c = c2
            last_ctrl_q = None
        elif cmd in ("Q", "q"):
            x1, y1, x, y = tokens[i : i + 4]
            i += 4
            base = 0j if cmd == "Q" else cur
            c1 = base + complex(x1, y1)
            p2 = base + complex(x, y)
            cur_pts.extend(_flatten_quad(cur, c1, p2, tol_user))
            cur = p2
            last_ctrl_q = c1
            last_ctrl_c = None
        elif cmd in ("T", "t"):
            x, y = tokens[i], tokens[i + 1]
            i += 2
            base = 0j if cmd == "T" else cur
            c1 = (2 * cur - last_ctrl_q) if last_ctrl_q is not None else cur
            p2 = base + complex(x, y)
            cur_pts.extend(_flatten_quad(cur, c1, p2, tol_user))
            cur = p2
            last_ctrl_q = c1
            last_ctrl_c = None
        elif cmd in ("A", "a"):
            rx, ry, rot, large_arc_f, sweep_f, x, y = tokens[i : i + 7]
            i += 7
            base = 0j if cmd == "A" else cur
            p1 = base + complex(x, y)
            cur_pts.extend(
                _flatten_arc(
                    cur,
                    rx,
                    ry,
                    rot,
                    bool(int(large_arc_f)),
                    bool(int(sweep_f)),
                    p1,
                    tol_user,
                )
            )
            cur = p1
            last_ctrl_c = None
            last_ctrl_q = None
        else:
            break

    if cur_pts:
        _close_subpath(closed=False)
    return subpaths


def _stroke_polyline_to_polygons(
    pts: np.ndarray,
    stroke_width: float,
    closed: bool,
    linecap: str = "round",
    tol_mm: float = 0.005,
) -> list[np.ndarray]:
    """Expand an open/closed 2D polyline of width ``stroke_width`` into closed polygons."""
    if len(pts) < 2 or stroke_width <= 0:
        return []
    half_w = 0.5 * stroke_width
    polys: list[np.ndarray] = []
    n_pts = len(pts)

    # Deduplicate consecutive identical points
    clean = [pts[0]]
    for pt in pts[1:]:
        if np.hypot(pt[0] - clean[-1][0], pt[1] - clean[-1][1]) > 1e-9:
            clean.append(pt)
    if (
        closed
        and len(clean) > 2
        and np.hypot(clean[-1][0] - clean[0][0], clean[-1][1] - clean[0][1]) <= 1e-9
    ):
        clean.pop()
    if len(clean) < 2:
        return []
    arr = np.array(clean, dtype=np.float64)
    n_pts = len(arr)

    def _cap_pts(
        center: np.ndarray, direction: np.ndarray, out_side: bool
    ) -> list[np.ndarray]:
        """Generate end-cap points between the two normals at an endpoint."""
        ux, uy = direction
        nx, ny = -uy, ux  # left normal
        a_start = math.atan2(ny, nx) if out_side else math.atan2(-ny, -nx)
        if linecap == "round":
            n_seg = max(4, math.ceil(math.pi * half_w / max(tol_mm, 0.002)))
            # Trace semicircle centered on center around tip direction
            step = -math.pi / n_seg
            return [
                center
                + half_w
                * np.array([math.cos(a_start + k * step), math.sin(a_start + k * step)])
                for k in range(n_seg + 1)
            ]
        elif linecap == "square":
            ext = center + direction * half_w
            sign = 1.0 if out_side else -1.0
            return [
                ext + sign * half_w * np.array([nx, ny]),
                ext - sign * half_w * np.array([nx, ny]),
            ]
        else:  # butt
            sign = 1.0 if out_side else -1.0
            return [
                center + sign * half_w * np.array([nx, ny]),
                center - sign * half_w * np.array([nx, ny]),
            ]

    # Emit one capsule/quad per segment cleanly (unioned per scanline via VectorShape)
    seg_count = n_pts if closed else (n_pts - 1)
    for i in range(seg_count):
        p0 = arr[i]
        p1 = arr[(i + 1) % n_pts]
        d = p1 - p0
        seg_len = float(np.hypot(d[0], d[1]))
        if seg_len < 1e-9:
            continue
        u = d / seg_len
        cap1 = _cap_pts(p1, u, out_side=True)
        cap0 = _cap_pts(p0, -u, out_side=True)
        polys.append(np.array(cap1 + cap0, dtype=np.float64))
    return polys


def _offset_contour(pts: np.ndarray, delta_mm: float) -> np.ndarray | None:
    """Offset a simple closed 2D polygon outward (>0) or inward (<0) by ``delta_mm``."""
    if len(pts) < 3 or abs(delta_mm) < 1e-9:
        return pts
    # Compute signed area (shoelace) to determine CCW vs CW orientation
    x, y = pts[:, 0], pts[:, 1]
    area2 = float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if abs(area2) < 1e-12:
        return None
    orient = 1.0 if area2 > 0 else -1.0

    n = len(pts)
    out = []
    miter_limit = 4.0 * abs(delta_mm)
    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p_curr = pts[i]
        p_next = pts[(i + 1) % n]
        e1 = p_curr - p_prev
        e2 = p_next - p_curr
        l1 = float(np.hypot(e1[0], e1[1]))
        l2 = float(np.hypot(e2[0], e2[1]))
        if l1 < 1e-12 or l2 < 1e-12:
            out.append(p_curr)
            continue
        e1 /= l1
        e2 /= l2
        # Outward unit normal for CCW polygon is (ey, -ex) * orient
        n1 = orient * np.array([e1[1], -e1[0]])
        n2 = orient * np.array([e2[1], -e2[0]])
        bis = n1 + n2
        bis_len2 = float(bis[0] ** 2 + bis[1] ** 2)
        if bis_len2 < 1e-10:
            out.append(p_curr + delta_mm * n1)
            continue
        denom = float(np.dot(bis, n1))
        scale = delta_mm / denom if abs(denom) > 1e-6 else delta_mm
        shift = (bis / math.sqrt(bis_len2)) * (scale * math.sqrt(bis_len2))
        shift_len = float(np.hypot(shift[0], shift[1]))
        if shift_len > miter_limit:
            shift *= miter_limit / shift_len
        out.append(p_curr + shift)
    return np.array(out, dtype=np.float64)


def parse_svg_polygons(
    svg_path: str | Path,
    curve_tolerance_mm: float = 0.002,
    spot_compensation_mm: float = 0.0,
) -> tuple[list[VectorShape], float, float]:
    """Parse an SVG file into closed 2D polygon :class:`VectorShape` objects in mm.

    Coordinates are returned in the Hexastorm bottom-up Design frame
    ``[0, width_mm] x [0, height_mm]`` (matching :meth:`Interpolator.svgtopil`).
    """
    tree = ET.parse(str(svg_path))
    root = tree.getroot()

    raw_w = root.attrib.get("width")
    raw_h = root.attrib.get("height")
    viewbox = root.attrib.get("viewBox")

    if viewbox:
        vb = [float(x) for x in _NUM_RE.findall(viewbox)]
        vb_x, vb_y, vb_w, vb_h = vb[0], vb[1], vb[2], vb[3]
    else:
        vb_x = vb_y = 0.0
        vb_w = _parse_length_to_user(raw_w, 100.0)
        vb_h = _parse_length_to_user(raw_h, 100.0)

    if raw_w and raw_h:
        width_mm = _parse_length_to_mm(raw_w, user_to_mm=_UNIT_TO_MM[""])
        height_mm = _parse_length_to_mm(raw_h, user_to_mm=_UNIT_TO_MM[""])
    else:
        width_mm = vb_w * _UNIT_TO_MM[""]
        height_mm = vb_h * _UNIT_TO_MM[""]

    sx = width_mm / vb_w if vb_w > 0 else 1.0
    sy = height_mm / vb_h if vb_h > 0 else 1.0
    avg_scale = 0.5 * (sx + sy)

    # Build root transform from viewBox -> mm
    root_mat = np.array(
        [[sx, 0.0, -vb_x * sx], [0.0, sy, -vb_y * sy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    # Index elements with `id` for `<use xlink:href="#...">`
    id_map: dict[str, ET.Element] = {}
    for el in root.iter():
        el_id = el.attrib.get("id")
        if el_id:
            id_map[el_id] = el

    shapes: list[VectorShape] = []
    default_style = {
        "fill": "#000000",
        "fill-rule": "nonzero",
        "stroke": "none",
        "stroke-width": "1",
        "stroke-linecap": "butt",
        "display": "inline",
        "visibility": "visible",
    }

    def _to_design_mm(pts_svg_mm: np.ndarray) -> np.ndarray:
        out = pts_svg_mm.copy()
        out[:, 1] = height_mm - out[:, 1]
        return out

    def _emit_contours(
        subpaths: list[tuple[np.ndarray, bool]],
        mat_mm: np.ndarray,
        style: dict[str, str],
    ) -> None:
        fill_color = style.get("fill", "#000000")
        stroke_color = style.get("stroke", "none")
        fill_rule = style.get("fill-rule", "nonzero").lower()
        stroke_linecap = style.get("stroke-linecap", "round").lower()

        has_fill = not _is_none_color(fill_color) and not _is_white_color(fill_color)
        stroke_w_raw = style.get("stroke-width", "1")
        det_scale = math.sqrt(abs(np.linalg.det(mat_mm[:2, :2])))
        local_scale = det_scale / max(avg_scale, 1e-12)
        stroke_w_mm = (
            _parse_length_to_mm(stroke_w_raw, user_to_mm=avg_scale, default=0.0)
            * local_scale
        )
        has_stroke = (
            not _is_none_color(stroke_color)
            and not _is_white_color(stroke_color)
            and stroke_w_mm > 0.0
        )

        if has_fill:
            fill_contours: list[np.ndarray] = []
            for pts_u, _closed in subpaths:
                if len(pts_u) >= 3:
                    pts_mm = _to_design_mm(_apply_matrix(pts_u, mat_mm))
                    if abs(spot_compensation_mm) > 1e-9:
                        comp = _offset_contour(pts_mm, spot_compensation_mm)
                        if comp is not None:
                            pts_mm = comp
                    if len(pts_mm) >= 3:
                        fill_contours.append(pts_mm)
            if fill_contours:
                shapes.append(VectorShape(contours=fill_contours, fill_rule=fill_rule))

        if has_stroke:
            eff_w = max(1e-6, stroke_w_mm + 2.0 * spot_compensation_mm)
            for pts_u, closed in subpaths:
                if len(pts_u) >= 2:
                    pts_mm = _to_design_mm(_apply_matrix(pts_u, mat_mm))
                    ribbon_polys = _stroke_polyline_to_polygons(
                        pts_mm,
                        stroke_width=eff_w,
                        closed=closed,
                        linecap=stroke_linecap,
                        tol_mm=curve_tolerance_mm,
                    )
                    if ribbon_polys:
                        shapes.append(
                            VectorShape(contours=ribbon_polys, fill_rule="evenodd")
                        )

    def _visit(
        elem: ET.Element, parent_mat: np.ndarray, parent_style: dict[str, str]
    ) -> None:
        tag = elem.tag.split("}")[-1].lower()
        if tag in ("defs", "metadata", "title", "desc", "style", "clippath", "mask"):
            return

        style = _parse_style(elem, parent_style)
        if (
            style.get("display", "").lower() == "none"
            or style.get("visibility", "").lower() == "hidden"
        ):
            return

        local_mat = _parse_transform(elem.attrib.get("transform"))
        cur_mat = parent_mat @ local_mat

        if tag in ("svg", "g", "a"):
            for child in elem:
                _visit(child, cur_mat, style)
            return

        if tag == "use":
            href = (
                elem.attrib.get("{http://www.w3.org/1999/xlink}href")
                or elem.attrib.get("href")
                or ""
            )
            ref_id = href.lstrip("#")
            target = id_map.get(ref_id)
            if target is not None:
                ux = _parse_length_to_mm(
                    elem.attrib.get("x"), user_to_mm=1.0, default=0.0
                )
                uy = _parse_length_to_mm(
                    elem.attrib.get("y"), user_to_mm=1.0, default=0.0
                )
                t_use = np.array(
                    [[1.0, 0.0, ux], [0.0, 1.0, uy], [0.0, 0.0, 1.0]], dtype=np.float64
                )
                use_mat = cur_mat @ t_use
                t_tag = target.tag.split("}")[-1].lower()
                if t_tag in ("g", "svg", "symbol"):
                    for ch in target:
                        _visit(ch, use_mat, style)
                else:
                    _visit(target, use_mat, style)
            return

        if tag == "path":
            d_str = elem.attrib.get("d", "")
            if d_str:
                det_scale = math.sqrt(abs(np.linalg.det(cur_mat[:2, :2])))
                local_tol = max(1e-6, curve_tolerance_mm / max(det_scale, 1e-9))
                _emit_contours(_parse_svg_path_d(d_str, local_tol), cur_mat, style)
        elif tag == "rect":
            x = float(elem.attrib.get("x", 0.0))
            y = float(elem.attrib.get("y", 0.0))
            w = float(elem.attrib.get("width", 0.0))
            h = float(elem.attrib.get("height", 0.0))
            if w > 0 and h > 0:
                pts = np.array(
                    [[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.float64
                )
                _emit_contours([(pts, True)], cur_mat, style)
        elif tag in ("circle", "ellipse"):
            cx = float(elem.attrib.get("cx", 0.0))
            cy = float(elem.attrib.get("cy", 0.0))
            if tag == "circle":
                rx = ry = float(elem.attrib.get("r", 0.0))
            else:
                rx = float(elem.attrib.get("rx", 0.0))
                ry = float(elem.attrib.get("ry", 0.0))
            if rx > 0 and ry > 0:
                det_scale = math.sqrt(abs(np.linalg.det(cur_mat[:2, :2])))
                local_tol = max(1e-6, curve_tolerance_mm / max(det_scale, 1e-9))
                r_max = max(rx, ry)
                step = 2.0 * math.acos(
                    max(-1.0, min(1.0, 1.0 - min(0.5, local_tol / r_max)))
                )
                n_seg = max(16, math.ceil(2.0 * math.pi / max(0.02, step)))
                angles = np.linspace(0.0, 2.0 * math.pi, n_seg, endpoint=False)
                pts = np.column_stack(
                    (cx + rx * np.cos(angles), cy + ry * np.sin(angles))
                )
                _emit_contours([(pts, True)], cur_mat, style)
        elif tag in ("polygon", "polyline"):
            nums = [float(x) for x in _NUM_RE.findall(elem.attrib.get("points", ""))]
            if len(nums) >= 4:
                pts = np.array(nums[: (len(nums) // 2) * 2], dtype=np.float64).reshape(
                    -1, 2
                )
                _emit_contours([(pts, tag == "polygon")], cur_mat, style)
        elif tag == "line":
            x1 = float(elem.attrib.get("x1", 0.0))
            y1 = float(elem.attrib.get("y1", 0.0))
            x2 = float(elem.attrib.get("x2", 0.0))
            y2 = float(elem.attrib.get("y2", 0.0))
            pts = np.array([[x1, y1], [x2, y2]], dtype=np.float64)
            _emit_contours([(pts, False)], cur_mat, style)

    _visit(root, root_mat, default_style)
    return shapes, width_mm, height_mm


# ---------------------------------------------------------------------------
# Direct 2D Scanline-Polygon Intersection & Interval Algebra
# ---------------------------------------------------------------------------


def _build_shape_edges(shape: VectorShape) -> dict[str, np.ndarray | str] | None:
    """Precompute vectorized edge arrays for a :class:`VectorShape`."""
    u0_list, v0_list, u1_list, v1_list = [], [], [], []
    for contour in shape.contours:
        if len(contour) < 3:
            continue
        c = np.asarray(contour, dtype=np.float64)
        c_next = np.roll(c, -1, axis=0)
        # Keep non-degenerate segments
        keep = (np.abs(c_next[:, 0] - c[:, 0]) > 1e-12) | (
            np.abs(c_next[:, 1] - c[:, 1]) > 1e-12
        )
        if not np.any(keep):
            continue
        u0_list.append(c[keep, 0])
        v0_list.append(c[keep, 1])
        u1_list.append(c_next[keep, 0])
        v1_list.append(c_next[keep, 1])

    if not u0_list:
        return None
    u0 = np.concatenate(u0_list)
    v0 = np.concatenate(v0_list)
    u1 = np.concatenate(u1_list)
    v1 = np.concatenate(v1_list)
    return {
        "u0": u0,
        "v0": v0,
        "u1": u1,
        "v1": v1,
        "du": u1 - u0,
        "poly_umin": float(min(u0.min(), u1.min())),
        "poly_umax": float(max(u0.max(), u1.max())),
        "poly_vmin": float(min(v0.min(), v1.min())),
        "poly_vmax": float(max(v0.max(), v1.max())),
        "fill_rule": shape.fill_rule,
        "intensity": shape.intensity,
    }


def _intersect_shape_with_scanline(
    edge_data: dict[str, np.ndarray | str],
    u_start: float,
    v_start: float,
    slope: float,
) -> list[tuple[float, float]]:
    """Compute filled ``u`` intervals ``[(u_in, u_out), ...]`` for one shape along a tilted scanline."""
    u0 = edge_data["u0"]
    v0 = edge_data["v0"]
    u1 = edge_data["u1"]
    v1 = edge_data["v1"]
    du = edge_data["du"]

    # Signed normal offset relative to scanline w(u, v) = (v - v_start) - slope * (u - u_start)
    w0 = (v0 - v_start) - slope * (u0 - u_start)
    w1 = (v1 - v_start) - slope * (u1 - u_start)

    # Half-open crossing rule avoids double counting at shared vertices
    c0_le = w0 <= 0.0
    c1_le = w1 <= 0.0
    crossings = np.flatnonzero(c0_le != c1_le)
    if crossings.size < 2:
        return []

    dw = w1[crossings] - w0[crossings]
    alpha = -w0[crossings] / dw
    u_hits = u0[crossings] + alpha * du[crossings]

    order = np.argsort(u_hits, kind="mergesort")
    u_sorted = u_hits[order]

    fill_rule = edge_data["fill_rule"]
    if fill_rule == "evenodd":
        pairs = (u_sorted.size // 2) * 2
        if pairs == 0:
            return []
        intervals = []
        for i in range(0, pairs, 2):
            a, b = float(u_sorted[i]), float(u_sorted[i + 1])
            if b > a + 1e-9:
                intervals.append((a, b))
        return intervals

    # nonzero winding rule: downward crossing (+1) vs upward crossing (-1)
    # Side along scanline increases in u, so crossing from w<=0 to w>0 has wind=+1
    winds = np.where(c0_le[crossings], 1, -1)[order]
    intervals = []
    acc = 0
    enter_u = 0.0
    for u_val, w_val in zip(u_sorted.tolist(), winds.tolist()):
        prev_acc = acc
        acc += w_val
        if prev_acc == 0 and acc != 0:
            enter_u = u_val
        elif prev_acc != 0 and acc == 0 and u_val > enter_u + 1e-9:
            intervals.append((enter_u, u_val))
    return intervals


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Return sorted disjoint union of 1D ``(start, end)`` intervals."""
    if not intervals:
        return []
    intervals.sort(key=lambda iv: iv[0])
    merged: list[tuple[float, float]] = [intervals[0]]
    for start, end in intervals[1:]:
        prev_s, prev_e = merged[-1]
        if start <= prev_e + 1e-9:
            if end > prev_e:
                merged[-1] = (prev_s, end)
        else:
            merged.append((start, end))
    return merged


def _invert_intervals(
    intervals: list[tuple[float, float]], bound_min: float, bound_max: float
) -> list[tuple[float, float]]:
    """Invert ``(start, end)`` intervals against ``[bound_min, bound_max]``."""
    out: list[tuple[float, float]] = []
    cursor = bound_min
    for a, b in intervals:
        if b <= cursor:
            continue
        if a > cursor:
            right = min(a, bound_max)
            if right > cursor + 1e-9:
                out.append((cursor, right))
        cursor = max(cursor, b)
        if cursor >= bound_max:
            break
    if cursor < bound_max - 1e-9:
        out.append((cursor, bound_max))
    return out


class VectorExposure:
    """Direct Vector-to-Timestamp exposure pipeline replacing bitmap rasterization for vector/PCB designs.

    Intersects closed polygon contours directly with each tilted laser scanline
    ``(u(t), v(t))`` and converts intersection coordinates into 24-bit clock ticks via
    :class:`PrismCurve`. Output files use the exact same compressed event-driven SPI
    format as :mod:`hexastorm.exposure.io`.
    """

    def __init__(
        self,
        correction: bool | dict = False,
        exposures: int = 1,
        mount: LaserheadMount | None = None,
        intensity: int = 255,
        placement: SubstratePlacement | None = None,
        machine: MachineFrame | None = None,
    ):
        self.cfg = PlatformConfig(test=False)
        self.correction = correction
        self.exposures = exposures
        self.mount = mount if mount is not None else LaserheadMount()
        self.placement = placement if placement is not None else SubstratePlacement()
        self.machine = (
            machine if machine is not None else MachineFrame.from_config(self.cfg)
        )
        self.default_intensity = max(1, min(255, int(intensity)))
        self.bitorder = "big"
        self._last_output_path: Path | None = None
        self._last_active_lines: int = 0
        self._last_coord: CoordinateTransform | None = None
        self.set_optical_params(correction=correction, exposures=exposures)

    def set_optical_params(
        self, correction: bool | dict | None = None, exposures: int | None = None
    ) -> None:
        if correction is not None:
            self.correction = correction
        if exposures is not None:
            self.exposures = exposures
        self.params = self.cfg.get_optical_params(
            correction=self.correction, exposures=self.exposures
        )
        self.curve = PrismCurve(
            self.params, invert_hw_scan_dir=self.mount.invert_hw_scan_dir
        )

    def slice_polygons(
        self,
        polygons: list[VectorShape | np.ndarray],
        width_mm: float,
        height_mm: float,
        filepath: str | Path,
        compression_level: int = 9,
        use_c: bool = True,
    ) -> Path:
        """Slice polygon shapes into an event-driven ``.pat`` / ``.bin`` binary file."""
        if width_mm <= 0 or height_mm <= 0:
            raise ValueError("Sample width_mm and height_mm must be positive")

        coord = CoordinateTransform(
            width_mm=width_mm,
            height_mm=height_mm,
            machine=self.machine,
            placement=self.placement,
            mount=self.mount,
        )
        coord.validate_fits_machine(
            max_x_mm=min(self.machine.x_size_mm, float(self.params["pltfxsize"])),
            max_y_mm=min(self.machine.y_size_mm, float(self.params["pltfysize"])),
        )
        self._last_coord = coord
        u_size_mm, v_size_mm = coord.u_size_mm, coord.v_size_mm

        # Normalize raw numpy contours to VectorShape and transform Substrate -> Laserhead
        transformed_shapes: list[VectorShape] = []
        for item in polygons:
            shape = (
                item
                if isinstance(item, VectorShape)
                else VectorShape(contours=[np.asarray(item, dtype=np.float64)])
            )
            t_contours = [coord.substrate_to_laserhead(cnt) for cnt in shape.contours]
            transformed_shapes.append(
                VectorShape(
                    contours=t_contours,
                    fill_rule=shape.fill_rule,
                    intensity=shape.intensity,
                )
            )

        params = self.params
        params["samplexsize"] = float(u_size_mm)
        params["sampleysize"] = float(v_size_mm)

        curve = self.curve
        lanewidth = curve.lanewidth
        facets = int(params["facets"])
        laser_hz = float(params["laser_hz"])
        stagespeed = float(params["stagespeed"])
        rot_freq = float(params["rotationfrequency"])
        stepsperline = float(params["stepsperline"])
        bits_in_scanline = curve.bits_in_scanline
        startpixel = curve.start_pixel
        positiveresist = bool(params["positiveresist"])

        lanes = math.ceil(u_size_mm / lanewidth)
        facets_inlane = math.ceil(rot_freq * facets * (v_size_mm / stagespeed))
        params["lanewidth"] = float(lanewidth)
        params["lanes"] = float(lanes)
        params["facetsinlane"] = float(facets_inlane)

        # Build edge structures once
        shape_edges = []
        for s in transformed_shapes:
            ed = _build_shape_edges(s)
            if ed is not None:
                shape_edges.append(ed)

        # Constants for scanline v_start / v_end trajectory
        y_shift_per_facet = stagespeed / (facets * rot_freq)
        p_start = float(bits_in_scanline - 1)
        p_end = 0.0
        v_prism_start = -curve.cos_tilt * curve.disp_last
        v_prism_end = -curve.cos_tilt * curve.disp_first
        v_stage_start = ((startpixel + p_start) / laser_hz) * stagespeed
        v_stage_end = ((startpixel + p_end) / laser_hz) * stagespeed
        slope_den = curve.raw_u_first - curve.raw_u_last

        out_path = Path(filepath)
        self._last_output_path = out_path
        active_lines = 0

        logger.info(f"Writing vector event-driven binary file to {out_path}...")
        with io.EventStreamWriter(
            filepath=out_path,
            lanewidth=lanewidth,
            facets_in_lane=facets_inlane,
            lanes=lanes,
            steps_per_line=stepsperline,
            bits_in_scanline=bits_in_scanline,
            compression_level=compression_level,
        ) as writer:
            if use_c and _HAS_C_CORE:
                if shape_edges:
                    u0_arr = np.ascontiguousarray(
                        np.concatenate([ed["u0"] for ed in shape_edges]),
                        dtype=np.float64,
                    )
                    v0_arr = np.ascontiguousarray(
                        np.concatenate([ed["v0"] for ed in shape_edges]),
                        dtype=np.float64,
                    )
                    u1_arr = np.ascontiguousarray(
                        np.concatenate([ed["u1"] for ed in shape_edges]),
                        dtype=np.float64,
                    )
                    v1_arr = np.ascontiguousarray(
                        np.concatenate([ed["v1"] for ed in shape_edges]),
                        dtype=np.float64,
                    )
                    du_arr = np.ascontiguousarray(
                        np.concatenate([ed["du"] for ed in shape_edges]),
                        dtype=np.float64,
                    )
                    edge_counts = np.array(
                        [ed["u0"].size for ed in shape_edges], dtype=np.int32
                    )
                    edge_starts = np.zeros(len(shape_edges), dtype=np.int32)
                    if len(shape_edges) > 1:
                        np.cumsum(edge_counts[:-1], out=edge_starts[1:])
                    poly_umin = np.array(
                        [ed["poly_umin"] for ed in shape_edges], dtype=np.float64
                    )
                    poly_umax = np.array(
                        [ed["poly_umax"] for ed in shape_edges], dtype=np.float64
                    )
                    poly_vmin = np.array(
                        [ed["poly_vmin"] for ed in shape_edges], dtype=np.float64
                    )
                    poly_vmax = np.array(
                        [ed["poly_vmax"] for ed in shape_edges], dtype=np.float64
                    )
                    fill_rules = np.array(
                        [
                            1 if ed["fill_rule"] == "nonzero" else 0
                            for ed in shape_edges
                        ],
                        dtype=np.uint8,
                    )
                else:
                    u0_arr = v0_arr = u1_arr = v1_arr = du_arr = np.zeros(
                        0, dtype=np.float64
                    )
                    edge_starts = edge_counts = np.zeros(0, dtype=np.int32)
                    poly_umin = poly_umax = poly_vmin = poly_vmax = np.zeros(
                        0, dtype=np.float64
                    )
                    fill_rules = np.zeros(0, dtype=np.uint8)

                dx_facet_arr = np.array(
                    [float(params[f"f{fi}_scan"]) for fi in range(facets)],
                    dtype=np.float64,
                )
                dy_facet_arr = np.array(
                    [float(params[f"f{fi}_orth"]) for fi in range(facets)],
                    dtype=np.float64,
                )
                u_samples = np.ascontiguousarray(
                    curve._u_base_samples, dtype=np.float64
                )
                tick_samples = np.ascontiguousarray(
                    curve._tick_samples, dtype=np.float64
                )
                scanline_cmd = int(Spi.Instructions.scanline)
                spi_write_cmd = int(Spi.Commands.write)

                for lane_idx in range(lanes):
                    raw_spi, lane_active = _vector_core.slice_lane_spi(
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
                        int(bits_in_scanline),
                        float(stepsperline),
                        float(u_size_mm),
                        float(v_size_mm),
                        float(lanewidth),
                        float(curve.raw_u_last),
                        float(curve.xstart_val),
                        float(slope_den),
                        float(y_shift_per_facet),
                        float(v_prism_start),
                        float(v_prism_end),
                        float(v_stage_start),
                        float(v_stage_end),
                        bool(positiveresist),
                        int(self.default_intensity),
                        scanline_cmd,
                        spi_write_cmd,
                    )
                    active_lines += lane_active
                    writer.write_raw_spi(raw_spi)
            else:
                dx_facets = [float(params[f"f{fi}_scan"]) for fi in range(facets)]
                dy_facets = [float(params[f"f{fi}_orth"]) for fi in range(facets)]
                min_dx = min(dx_facets)
                max_dx = max(dx_facets)

                for lane_idx in range(lanes):
                    direction = 1 if lane_idx % 2 == 0 else 0
                    is_forward = lane_idx % 2 == 0
                    dir_sign = 1.0 if is_forward else -1.0
                    lane_u_offset = lane_idx * lanewidth
                    min_u_s = (
                        curve.raw_u_last + curve.xstart_val + min_dx + lane_u_offset
                    )
                    max_u_s = (
                        curve.raw_u_last + curve.xstart_val + max_dx + lane_u_offset
                    )
                    lane_u_min = max(0.0, min(min_u_s, min_u_s + slope_den)) - 1e-6
                    lane_u_max = (
                        min(u_size_mm, max(max_u_s, max_u_s + slope_den)) + 1e-6
                    )
                    lane_shapes = (
                        [
                            ed
                            for ed in shape_edges
                            if ed["poly_umax"] >= lane_u_min
                            and ed["poly_umin"] <= lane_u_max
                        ]
                        if lane_u_max > lane_u_min
                        else []
                    )

                    for f_lane_idx in range(facets_inlane):
                        facet_idx = f_lane_idx % facets
                        dx_facet = dx_facets[facet_idx]
                        dy_facet = dy_facets[facet_idx]

                        ystart_mm = (
                            f_lane_idx * y_shift_per_facet
                            if is_forward
                            else (facets_inlane - f_lane_idx) * y_shift_per_facet
                        )

                        # Scanline sweep start/end in laserhead coordinates
                        # Note: geometry.py sets u(p) = raw_u(p) + xstart_val + dx_facet + l_idx*lanewidth
                        u_shift = (
                            (curve.raw_u_last + curve.xstart_val)
                            + dx_facet
                            + lane_u_offset
                        )
                        u_start = u_shift  # corresponds to u_base = 0 (p = N-1)
                        u_end = (
                            u_shift + slope_den
                        )  # corresponds to u_base = lanewidth (p = 0)

                        # Window clip bounds matching geometry/interpolator sample [0, u_size]
                        win_u_min = max(0.0, u_start)
                        win_u_max = min(u_size_mm, u_end)

                        v_start = (
                            v_prism_start
                            + dy_facet
                            + dir_sign * v_stage_start
                            + ystart_mm
                        )
                        v_end = (
                            v_prism_end + dy_facet + dir_sign * v_stage_end + ystart_mm
                        )
                        slope = (
                            (v_end - v_start) / slope_den
                            if abs(slope_den) > 1e-12
                            else 0.0
                        )

                        # 1. Intersect all active shapes along scanline (u, v)
                        all_intervals: list[tuple[float, float]] = []
                        in_v_bounds = (
                            max(v_start, v_end) >= 0.0
                            and min(v_start, v_end) < v_size_mm
                        )
                        if (
                            win_u_max > win_u_min
                            and (in_v_bounds or positiveresist)
                            and lane_shapes
                        ):
                            for ed in lane_shapes:
                                v_u0 = v_start + slope * (ed["poly_umin"] - u_start)
                                v_u1 = v_start + slope * (ed["poly_umax"] - u_start)
                                if (
                                    ed["poly_vmax"] < min(v_u0, v_u1) - 1e-9
                                    or ed["poly_vmin"] > max(v_u0, v_u1) + 1e-9
                                ):
                                    continue
                                ivs = _intersect_shape_with_scanline(
                                    ed, u_start, v_start, slope
                                )
                                if ivs:
                                    all_intervals.extend(ivs)

                        merged = _merge_intervals(all_intervals)
                        if positiveresist:
                            merged = _invert_intervals(merged, 0.0, u_size_mm)

                        # 2. Clip to active laser scanline window and convert u -> ticks
                        tick_intervals: list[tuple[int, int]] = []
                        for a, b in merged:
                            ca = max(a, win_u_min)
                            cb = min(b, win_u_max)
                            if cb <= ca:
                                continue
                            t_a = curve.u_to_tick(ca - u_shift)
                            t_b = curve.u_to_tick(cb - u_shift)
                            t_on, t_off = (t_a, t_b) if t_a <= t_b else (t_b, t_a)
                            if t_off > t_on:
                                tick_intervals.append((t_on, t_off))

                        # Merge adjacent tick intervals (e.g. if sub-tick gap rounds to 0)
                        events: list[tuple[int, int]] = []
                        if tick_intervals:
                            tick_intervals.sort()
                            cur_on, cur_off = tick_intervals[0]
                            for t_on, t_off in tick_intervals[1:]:
                                if t_on <= cur_off:
                                    cur_off = max(cur_off, t_off)
                                else:
                                    events.append((cur_on, self.default_intensity))
                                    events.append((cur_off, 0))
                                    cur_on, cur_off = t_on, t_off
                            events.append((cur_on, self.default_intensity))
                            events.append((cur_off, 0))

                        if events:
                            active_lines += 1
                        writer.write_scanline(events, direction)

        self._last_active_lines = active_lines
        return out_path

    def _tagged_bin_path(self, path: Path) -> Path:
        """Append exposure count and optical-correction tag matching RasterExposure."""
        return io.tagged_bin_path(path, self.params)

    def svg_to_bin(
        self,
        svg_path: str | Path,
        bin_path: str | Path | None = None,
        curve_tolerance_mm: float = 0.002,
        spot_compensation_mm: float = 0.0,
        compression_level: int = 9,
    ) -> Path:
        """Parse an SVG vector file and directly compile it to a ``.pat`` / ``.bin`` file."""
        in_path = Path(svg_path)
        if not in_path.is_absolute() and not in_path.exists():
            in_path = self.cfg.paths["svgs"] / in_path
        shapes, width_mm, height_mm = parse_svg_polygons(
            in_path,
            curve_tolerance_mm=curve_tolerance_mm,
            spot_compensation_mm=spot_compensation_mm,
        )
        if not shapes:
            raise ValueError(f"No vector geometry found in {in_path}")

        if bin_path is None:
            out_path = self.cfg.paths["patterns"] / in_path.with_suffix(".pat").name
        else:
            out_path = Path(bin_path)
            if not out_path.is_absolute():
                out_path = self.cfg.paths["patterns"] / out_path

        return self.slice_polygons(
            polygons=shapes,
            width_mm=width_mm,
            height_mm=height_mm,
            filepath=out_path,
            compression_level=compression_level,
        )

    def img_to_bin(
        self,
        img_name: str | Path,
        bin_name: str | Path | None = None,
        camera: bool = False,
        curve_tolerance_mm: float = 0.002,
        spot_compensation_mm: float = 0.0,
    ) -> float:
        """Convert SVG directly to tagged ``.pat``/``.bin`` and return job duration in minutes."""
        img_path = Path(img_name)
        out_target = (
            img_path.with_suffix(".pat") if bin_name is None else Path(bin_name)
        )
        if not out_target.is_absolute():
            out_target = self.cfg.paths["patterns"] / out_target
        tagged_out = self._tagged_bin_path(out_target)

        self.svg_to_bin(
            img_path,
            bin_path=tagged_out,
            curve_tolerance_mm=curve_tolerance_mm,
            spot_compensation_mm=spot_compensation_mm,
        )

        if camera:
            total_min = self._last_active_lines / 180.0
        else:
            laz_tim = self.cfg.laser_timing
            lanes = self.params["lanes"]
            facets = self.params["facetsinlane"]
            line_min = laz_tim["facets"] * laz_tim["rpm"]
            expose_time = (lanes * facets) / line_min
            total_min = 0.8 + expose_time + lanes * 0.2

        device = "camera" if camera else "machine"
        logger.info(f"Estimated Job Duration: {total_min:.2f} minutes using {device}")
        return total_min

    def readbin(self, filename: str | Path | None = None) -> dict:
        """Read back an event-driven ``.bin`` / ``.pat`` file via :mod:`hexastorm.exposure.io`."""
        if filename is None:
            if self._last_output_path is None:
                raise ValueError("No output path specified or recorded yet")
            filename = self._last_output_path
        in_path = Path(filename)
        if not in_path.is_absolute() and not in_path.exists():
            in_path = self.cfg.paths["patterns"] / in_path
        if not in_path.exists():
            tagged_path = self._tagged_bin_path(in_path)
            if tagged_path.exists():
                in_path = tagged_path
        facets, lanes, width, data_bytes = io.read_binary_file(
            in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
        )
        data_bits = np.unpackbits(data_bytes, bitorder=self.bitorder)
        return {
            "data": data_bits,
            "metadata": {
                "lanes": lanes,
                "lanewidth": width,
                "facetsinlane": facets,
            },
        }

    def plotptrn(
        self,
        ptrn_data: np.ndarray | dict | None = None,
        filename: str = "plot",
        color_lanes: bool = True,
        view: str = "substrate",
    ) -> np.ndarray:
        """Visualize the generated event-driven pattern via :mod:`hexastorm.exposure.plotter`."""
        if ptrn_data is None:
            ptrn_data = self.readbin()
        return plotter.plot_pattern(
            ptrn_data,
            cfg=self.cfg,
            correction=self.correction,
            exposures=self.exposures,
            filename=filename,
            color_lanes=color_lanes,
            view=view,
            machine=self.machine,
            coord=self._last_coord,
        )
