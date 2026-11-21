"""Shared MicroPython (ulab) and CPython (NumPy) calibration core.

Contains all dependency-light math, optical prism mapping, 2D spot centroiding,
global rotation rectification, spatial-median facet shift estimation, analytical
24 MHz dot-grid slicing, and 2D virtual frame stacking used by both PC and
ESP32-S3 calibration routines.
"""

import math
import sys

if sys.implementation.name == "micropython":
    from ulab import numpy as np

    _NP_FLOAT = np.float
else:
    import numpy as np

    _NP_FLOAT = float


def _displacement_scalar(frac: float, facets: float, inradius: float, n: float) -> float:
    """Compute scalar Snell's-law prism displacement [mm] at facet fraction."""
    i_max = 180.0 / facets
    angle = math.radians(-2.0 * i_max * frac + i_max)
    sin_a = math.sin(angle)
    sin_a2 = sin_a * sin_a
    n2 = n * n
    thickness = inradius * 2.0
    return thickness * sin_a * (1.0 - math.sqrt((1.0 - sin_a2) / (n2 - sin_a2)))


class PrismCurveLite:
    """MicroPython-safe 1D Snell's-law LUT mapping lane-local u [mm] <-> 24 MHz ticks."""

    def __init__(self, params: dict, invert_hw_scan_dir: bool = True, n_lut: int = 65):
        self.total_ticks = int(params["scanline_ticks"])
        self.start_frac = float(params["start_frac"])
        self.end_frac = float(params["end_frac"])
        self.tiltangle = float(params.get("tiltangle", math.pi * 0.5))
        self.sin_tilt = math.sin(self.tiltangle)
        self.cos_tilt = math.cos(self.tiltangle)
        self.invert_hw_scan_dir = bool(invert_hw_scan_dir)
        self.f0_scan = float(params.get("f0_scan", 0.0))

        facets = float(params["facets"])
        inradius = float(params.get("inradius", 15.0))
        n_idx = float(params.get("n", 1.49))

        n_pts = max(9, int(n_lut))
        raw_u = []
        disp_vals = []
        ticks = []
        for i in range(n_pts):
            alpha = i / (n_pts - 1)
            frac = self.end_frac + alpha * (self.start_frac - self.end_frac)
            disp = _displacement_scalar(frac, facets, inradius, n_idx)
            disp_vals.append(disp)
            raw_u.append(self.sin_tilt * disp)
            ticks.append(alpha * float(self.total_ticks))

        self.raw_u_last = raw_u[0]
        self.raw_u_first = raw_u[-1]
        self.xstart_val = abs(self.raw_u_last + self.f0_scan)
        self.lanewidth = abs(self.raw_u_first - self.raw_u_last)

        u_base = [u - self.raw_u_last for u in raw_u]
        if u_base[0] > u_base[-1]:
            u_base.reverse()
            ticks.reverse()

        self._u_samples = u_base
        self._t_samples = ticks
        if ticks[0] <= ticks[-1]:
            self._ts_asc = ticks
            self._us_asc = u_base
        else:
            self._ts_asc = list(reversed(ticks))
            self._us_asc = list(reversed(u_base))

    @staticmethod
    def _interp_sorted(x: float, xs: list[float], ys: list[float]) -> float:
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        lo = 0
        hi = len(xs) - 1
        while hi - lo > 1:
            mid = (lo + hi) >> 1
            if xs[mid] <= x:
                lo = mid
            else:
                hi = mid
        span = xs[hi] - xs[lo]
        if abs(span) < 1e-15:
            return ys[lo]
        frac = (x - xs[lo]) / span
        return ys[lo] + frac * (ys[hi] - ys[lo])

    def u_to_tick(self, u_base_mm: float) -> int:
        """Convert lane-local u [mm] to integer 24 MHz clock tick in [0, total_ticks]."""
        t = self._interp_sorted(float(u_base_mm), self._u_samples, self._t_samples)
        if not self.invert_hw_scan_dir:
            t = float(self.total_ticks) - t
        tick = round(t)
        if tick < 0:
            return 0
        if tick > self.total_ticks:
            return self.total_ticks
        return tick

    def tick_to_u(self, tick: float) -> float:
        """Convert 24 MHz clock tick back to lane-local u_base [mm]."""
        t = float(tick)
        if not self.invert_hw_scan_dir:
            t = float(self.total_ticks) - t
        return self._interp_sorted(t, self._ts_asc, self._us_asc)


def _median_list(values: list[float]) -> float:
    """Compute exact median of a non-empty Python list without CPython-only np.median."""
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return float(s[mid])
    return 0.5 * (float(s[mid - 1]) + float(s[mid]))


def subtract_dark_frame_u8(img, dark_frame=None):
    """Clamped uint8 dark-frame subtraction without int16/int32 allocation."""
    if dark_frame is not None and dark_frame.shape == img.shape:
        return np.maximum(img, dark_frame) - dark_frame
    return img


def get_dots_ulab(
    img,
    dark_frame=None,
    pixelsize: float = 4.4,
    min_diameter_um: float = 10.0,
    max_diameter_um: float = 250.0,
    threshold: int | None = None,
    gap_tol: int = 2,
):
    """Detect laser spots and compute sub-pixel intensity-weighted centroids using ulab.

    Returns ``(centroids_Nx2, metrics_dict)`` sorted along the dominant axis.
    """
    clean = subtract_dark_frame_u8(img, dark_frame)
    row_max = np.max(clean, axis=1)
    peak = int(np.max(row_max))

    empty_metrics = {
        "mean_spot_size_um": 0.0,
        "mean_eccentricity": 0.0,
        "num_dots": 0,
        "peak_intensity": peak,
    }
    if peak < 20:
        return np.zeros((0, 2), dtype=_NP_FLOAT), empty_metrics

    thresh = int(threshold) if threshold is not None else max(25, int(peak * 0.45))
    col_max = np.max(clean, axis=0)

    active_rows = [y for y in range(clean.shape[0]) if int(row_max[y]) >= thresh]
    active_cols = [x for x in range(clean.shape[1]) if int(col_max[x]) >= thresh]
    if not active_rows or not active_cols:
        return np.zeros((0, 2), dtype=_NP_FLOAT), empty_metrics

    x_min = max(0, active_cols[0] - 1)
    x_max = min(clean.shape[1] - 1, active_cols[-1] + 1)

    # cid -> [area, w_sum, wx_sum, wy_sum, wxx_sum, wyy_sum, wxy_sum, parent_cid]
    comps: dict[int, list] = {}
    next_cid = 0
    recent_rows: list[tuple[int, list]] = []

    def find_root(cid: int) -> int:
        while comps[cid][7] != cid:
            comps[cid][7] = comps[comps[cid][7]][7]
            cid = comps[cid][7]
        return cid

    def union_cids(cid_a: int, cid_b: int) -> int:
        ra = find_root(cid_a)
        rb = find_root(cid_b)
        if ra == rb:
            return ra
        if ra > rb:
            ra, rb = rb, ra
        ca = comps[ra]
        cb = comps[rb]
        for k in range(7):
            ca[k] += cb[k]
        cb[7] = ra
        return ra

    for y in active_rows:
        recent_rows = [item for item in recent_rows if (y - item[0]) <= gap_tol]
        row_slice = memoryview(clean[y, x_min : x_max + 1])

        runs = []
        in_run = False
        r_start = 0
        ra = 0
        rw = rwx = rwy = rwxx = rwyy = rwxy = 0.0
        fy = float(y)

        for idx, val in enumerate(row_slice):
            if val >= thresh:
                x = x_min + idx
                fx = float(x)
                fv = float(val)
                if not in_run:
                    in_run = True
                    r_start = x
                    ra = 1
                    rw = fv
                    rwx = fv * fx
                    rwy = fv * fy
                    rwxx = fv * fx * fx
                    rwyy = fv * fy * fy
                    rwxy = fv * fx * fy
                else:
                    ra += 1
                    rw += fv
                    rwx += fv * fx
                    rwy += fv * fy
                    rwxx += fv * fx * fx
                    rwyy += fv * fy * fy
                    rwxy += fv * fx * fy
            elif in_run:
                in_run = False
                r_end = x_min + idx - 1
                if runs and (r_start - runs[-1][1]) <= gap_tol:
                    prev = runs[-1]
                    prev[1] = r_end
                    prev[2] += ra
                    prev[3] += rw
                    prev[4] += rwx
                    prev[5] += rwy
                    prev[6] += rwxx
                    prev[7] += rwyy
                    prev[8] += rwxy
                else:
                    runs.append(
                        [r_start, r_end, ra, rw, rwx, rwy, rwxx, rwyy, rwxy, -1]
                    )

        if in_run:
            r_end = x_max
            if runs and (r_start - runs[-1][1]) <= gap_tol:
                prev = runs[-1]
                prev[1] = r_end
                prev[2] += ra
                prev[3] += rw
                prev[4] += rwx
                prev[5] += rwy
                prev[6] += rwxx
                prev[7] += rwyy
                prev[8] += rwxy
            else:
                runs.append([r_start, r_end, ra, rw, rwx, rwy, rwxx, rwyy, rwxy, -1])

        cur_row_runs = []
        for r in runs:
            x0, x1, ra, rw, rwx, rwy, rwxx, rwyy, rwxy, _ = r
            matched_root = -1
            for _, prev_runs in recent_rows:
                for pr in prev_runs:
                    if pr[0] - gap_tol <= x1 and pr[1] + gap_tol >= x0:
                        proot = find_root(pr[2])
                        matched_root = (
                            proot if matched_root < 0 else union_cids(matched_root, proot)
                        )

            if matched_root < 0:
                cid = next_cid
                next_cid += 1
                comps[cid] = [ra, rw, rwx, rwy, rwxx, rwyy, rwxy, cid]
                matched_root = cid
            else:
                c = comps[matched_root]
                c[0] += ra
                c[1] += rw
                c[2] += rwx
                c[3] += rwy
                c[4] += rwxx
                c[5] += rwyy
                c[6] += rwxy

            cur_row_runs.append([x0, x1, matched_root])

        recent_rows.append((y, cur_row_runs))

    min_radius_px = (min_diameter_um / 2.0) / pixelsize
    max_radius_px = (max_diameter_um / 2.0) / pixelsize
    min_area = max(3, int(math.pi * (min_radius_px**2)))
    max_area = max(min_area + 10, int(math.pi * (max_radius_px**2)))

    valid_dots = []
    spot_sizes_um = []
    eccentricities = []

    for cid, c in comps.items():
        if c[7] != cid:
            continue
        area, w_sum, wx_sum, wy_sum, wxx_sum, wyy_sum, wxy_sum, _ = c
        if area < min_area or area > max_area or w_sum <= 0:
            continue

        cx = wx_sum / w_sum
        cy = wy_sum / w_sum
        valid_dots.append((cx, cy))

        area_um2 = area * (pixelsize**2)
        spot_sizes_um.append(2.0 * math.sqrt(area_um2 / math.pi))

        mu20 = max(0.0, (wxx_sum / w_sum) - (cx * cx))
        mu02 = max(0.0, (wyy_sum / w_sum) - (cy * cy))
        mu11 = (wxy_sum / w_sum) - (cx * cy)
        common = math.sqrt(((mu20 - mu02) ** 2) + 4.0 * (mu11**2))
        l1 = 0.5 * (mu20 + mu02 + common)
        l2 = max(0.0, 0.5 * (mu20 + mu02 - common))
        ecc = math.sqrt(max(0.0, 1.0 - (l2 / l1))) if l1 > 1e-9 else 0.0
        eccentricities.append(ecc)

    if not valid_dots:
        return np.zeros((0, 2), dtype=_NP_FLOAT), empty_metrics

    if len(valid_dots) > 1:
        xs = [pt[0] for pt in valid_dots]
        ys = [pt[1] for pt in valid_dots]
        if (max(ys) - min(ys)) > (max(xs) - min(xs)):
            valid_dots.sort(key=lambda pt: pt[1])
        else:
            valid_dots.sort(key=lambda pt: pt[0])

    dots_arr = np.array(valid_dots, dtype=_NP_FLOAT)
    metrics = {
        "mean_spot_size_um": round(sum(spot_sizes_um) / len(spot_sizes_um), 2),
        "mean_eccentricity": round(sum(eccentricities) / len(eccentricities), 3),
        "num_dots": len(valid_dots),
        "peak_intensity": peak,
    }
    return dots_arr, metrics


def compute_angle_ulab(dots) -> float:
    """Compute scanline tilt angle [deg] via linear regression in ulab."""
    if len(dots.shape) != 2 or dots.shape[0] < 2:
        return 0.0
    x = dots[:, 0]
    y = dots[:, 1]
    xm = float(np.mean(x))
    ym = float(np.mean(y))
    dx = x - xm
    dy = y - ym
    sum_dx2 = float(np.sum(dx * dx))
    sum_dy2 = float(np.sum(dy * dy))
    if sum_dy2 > sum_dx2:
        if abs(sum_dy2) < 1e-12:
            return 90.0
        slope_x = float(np.sum(dy * dx)) / sum_dy2
        return math.degrees(math.atan2(1.0, slope_x))
    if abs(sum_dx2) < 1e-12:
        return 0.0
    slope = float(np.sum(dx * dy)) / sum_dx2
    return math.degrees(math.atan(slope))


def rotate_dots_ulab(dots, angle_deg: float):
    """Rotate 2D dot coordinates (N, 2) by -angle_deg around the origin."""
    if len(dots.shape) != 2 or dots.shape[0] == 0:
        return np.zeros((0, 2), dtype=_NP_FLOAT)
    n = dots.shape[0]
    theta = math.radians(-float(angle_deg))
    c = math.cos(theta)
    s = math.sin(theta)
    x = dots[:, 0]
    y = dots[:, 1]
    out = np.zeros((n, 2), dtype=_NP_FLOAT)
    out[:, 0] = x * c - y * s
    out[:, 1] = x * s + y * c
    return out


def calculate_facet_shifts_ulab(
    rect_dots_list,
    pixelsize: float = 4.4,
    max_shift_um: float = 250.0,
):
    """Calculate per-facet shifts (target - virtual_reference) using spatial-median reference.

    Sign convention matches ``hexastorm.calibration.analyzer.calculate_facet_shifts``:
    a positive shift means that facet's spot lands at a larger coordinate than the
    virtual reference facet, and feeding that offset into ``get_optical_params`` /
    slicers compensates it cleanly.
    """
    num_facets = len(rect_dots_list)
    if num_facets == 0 or any(d.shape[0] < 2 for d in rect_dots_list):
        raise ValueError("Not enough valid dots across facets to calculate shifts.")

    px_um = abs(float(pixelsize))
    max_shift_px = float(max_shift_um) / px_um

    # Convert to Python (x, y) tuples for compact ulab/CPython dot matching
    facets_pts = [
        [(float(d[k, 0]), float(d[k, 1])) for k in range(d.shape[0])]
        for d in rect_dots_list
    ]
    anchor_idx = max(range(num_facets), key=lambda i: len(facets_pts[i]))

    virtual_ref = []
    matched_by_facet = {i: [] for i in range(num_facets)}

    for ax, ay in facets_pts[anchor_idx]:
        group = []
        valid = True
        for f_idx in range(num_facets):
            best_pt = None
            best_d = max_shift_px + 1.0
            for px, py in facets_pts[f_idx]:
                dist = math.hypot(px - ax, py - ay)
                if dist <= max_shift_px and dist < best_d:
                    best_d = dist
                    best_pt = (px, py)
            if best_pt is None:
                valid = False
                break
            group.append(best_pt)

        if valid:
            rx = _median_list([p[0] for p in group])
            ry = _median_list([p[1] for p in group])
            virtual_ref.append((rx, ry))
            for f_idx in range(num_facets):
                matched_by_facet[f_idx].append(group[f_idx])

    if len(virtual_ref) < 2:
        raise ValueError(
            "Failed to build virtual reference: too few matching dots across all facets."
        )

    facet_data = {}
    n_matched = len(virtual_ref)
    for f_idx in range(num_facets):
        dx_um = [
            (matched_by_facet[f_idx][k][0] - virtual_ref[k][0]) * px_um
            for k in range(n_matched)
        ]
        dy_um = [
            (matched_by_facet[f_idx][k][1] - virtual_ref[k][1]) * px_um
            for k in range(n_matched)
        ]
        mean_dx = sum(dx_um) / n_matched
        mean_dy = sum(dy_um) / n_matched
        std_dx = math.sqrt(sum((v - mean_dx) ** 2 for v in dx_um) / n_matched)
        std_dy = math.sqrt(sum((v - mean_dy) ** 2 for v in dy_um) / n_matched)
        facet_data[str(f_idx)] = {
            "Scan_shift_um": round(mean_dx, 3),
            "Orth_shift_um": round(mean_dy, 3),
            "std_scan_um": round(std_dx, 3),
            "std_orth_um": round(std_dy, 3),
            "dots_matched": n_matched,
        }

    ref_arr = np.array(virtual_ref, dtype=_NP_FLOAT)
    return facet_data, ref_arr


def analyze_facets_ulab(
    facet_frames,
    dark_frame=None,
    pixelsize: float = 4.4,
    compute_rotation: bool = True,
    rotation_angle_deg: float | None = None,
    min_diameter_um: float = 10.0,
    max_diameter_um: float = 250.0,
    max_shift_um: float = 250.0,
):
    """Analyze facet frames in RAM using unified global median rotation & virtual reference."""
    raw_dots = []
    spot_metrics = []
    ind_angles = []

    for idx, frame in enumerate(facet_frames):
        dots, metrics = get_dots_ulab(
            frame,
            dark_frame=dark_frame,
            pixelsize=pixelsize,
            min_diameter_um=min_diameter_um,
            max_diameter_um=max_diameter_um,
        )
        if dots.shape[0] < 2:
            raise ValueError(
                f"Facet {idx}: insufficient calibration dots detected ({dots.shape[0]})."
            )
        raw_dots.append(dots)
        spot_metrics.append(metrics)
        if compute_rotation:
            ind_angles.append(compute_angle_ulab(dots))

    if compute_rotation:
        global_angle = _median_list(ind_angles)
        rect_dots = [rotate_dots_ulab(d, global_angle) for d in raw_dots]
    elif rotation_angle_deg is not None:
        global_angle = float(rotation_angle_deg)
        rect_dots = [rotate_dots_ulab(d, global_angle) for d in raw_dots]
    else:
        global_angle = 0.0
        rect_dots = raw_dots

    shift_data, _ref = calculate_facet_shifts_ulab(
        rect_dots, pixelsize=pixelsize, max_shift_um=max_shift_um
    )

    results = {}
    for idx in range(len(facet_frames)):
        f_key = str(idx)
        s_um = float(shift_data[f_key]["Scan_shift_um"])
        o_um = float(shift_data[f_key]["Orth_shift_um"])
        results[f_key] = {
            "Scan_shift_um": round(s_um, 2),
            "Orth_shift_um": round(o_um, 2),
            "Total_error_um": round(math.hypot(s_um, o_um), 2),
            "rotation_angle_deg": round(
                float(ind_angles[idx]) if compute_rotation else global_angle, 4
            ),
            "global_angle_deg": round(global_angle, 4),
            "mean_spot_size_um": spot_metrics[idx]["mean_spot_size_um"],
            "mean_eccentricity": spot_metrics[idx]["mean_eccentricity"],
            "num_dots": spot_metrics[idx]["num_dots"],
            "std_scan_um": shift_data[f_key]["std_scan_um"],
            "std_orth_um": shift_data[f_key]["std_orth_um"],
            "dots_matched": shift_data[f_key]["dots_matched"],
        }
    return results


def make_dot_centers_mm(
    u_min_mm: float,
    u_max_mm: float,
    num_dots: int = 6,
    margin_mm: float = 0.25,
) -> list[float]:
    """Compute evenly spaced physical dot X centers [mm] inside camera FOV bounds."""
    lo = float(u_min_mm) + float(margin_mm)
    hi = float(u_max_mm) - float(margin_mm)
    if hi <= lo:
        return [0.5 * (float(u_min_mm) + float(u_max_mm))]
    n = max(1, int(num_dots))
    if n == 1:
        return [0.5 * (lo + hi)]
    span = hi - lo
    return [lo + (i * span) / (n - 1) for i in range(n)]


def _intervals_to_events(
    intervals: list[tuple[float, float]],
    curve: PrismCurveLite,
    scan_offset_mm: float = 0.0,
    intensity: int = 255,
) -> list[tuple[int, int]]:
    """Merge continuous physical ``[u_a, u_b]`` intervals into strictly ascending clock-tick events."""
    tick_ivs: list[tuple[int, int]] = []
    lw = curve.lanewidth
    shift = float(scan_offset_mm)
    for a_raw, b_raw in intervals:
        a = max(0.0, float(a_raw) - shift)
        b = min(lw, float(b_raw) - shift)
        if b <= a:
            continue
        ta = curve.u_to_tick(a)
        tb = curve.u_to_tick(b)
        t_on, t_off = (ta, tb) if ta <= tb else (tb, ta)
        if t_off > t_on:
            tick_ivs.append((t_on, t_off))

    if not tick_ivs:
        return []
    tick_ivs.sort()
    merged: list[tuple[int, int]] = [tick_ivs[0]]
    for t_on, t_off in tick_ivs[1:]:
        prev_on, prev_off = merged[-1]
        if t_on <= prev_off:
            if t_off > prev_off:
                merged[-1] = (prev_on, t_off)
        else:
            merged.append((t_on, t_off))

    inten = max(1, min(255, int(intensity)))
    events: list[tuple[int, int]] = []
    total_ticks = curve.total_ticks
    for t_on, t_off in merged:
        if t_on >= total_ticks:
            continue
        events.append((t_on, inten))
        if t_off < total_ticks:
            events.append((t_off, 0))
    return events


def make_tick_dot_events(
    u_centers_mm: list[float],
    dot_width_mm: float,
    curve: PrismCurveLite,
    scan_offset_mm: float = 0.0,
    intensity: int = 255,
) -> list[tuple[int, int]]:
    """Slice 1D calibration dots directly into exact 24 MHz clock-tick events."""
    half_w = 0.5 * float(dot_width_mm)
    intervals = [(cx - half_w, cx + half_w) for cx in u_centers_mm]
    return _intervals_to_events(
        intervals, curve, scan_offset_mm=scan_offset_mm, intensity=intensity
    )


def slice_dot_grid_events(
    u_min_mm: float,
    u_max_mm: float,
    params: dict,
    num_dots_x: int = 6,
    num_dots_y: int = 1,
    dot_diameter_mm: float = 0.08,
    exposures: int = 4,
    steps_mm: float = 100.0,
    facet_offsets_mm: dict[int, tuple[float, float]] | None = None,
    invert_hw_scan_dir: bool = True,
    intensity: int = 255,
) -> dict:
    """Analytically slice a 2D circular dot grid into per-row 24 MHz tick events (no SVG/matplotlib).

    Returns ``{"rows": [(row_idx, facet_idx, events), ...], "total_rows": int,
    "y_step_mm": float, "num_facets": int, "dot_centers": list[tuple[float, float]]}``.
    """
    curve = PrismCurveLite(params, invert_hw_scan_dir=invert_hw_scan_dir)
    num_facets = int(params["facets"])
    y_step_mm = (1.0 / float(exposures)) / float(steps_mm)
    r = 0.5 * float(dot_diameter_mm)
    margin_mm = max(0.25, float(dot_diameter_mm))

    xs = make_dot_centers_mm(u_min_mm, u_max_mm, num_dots=num_dots_x, margin_mm=margin_mm)
    y_offset = 0.15
    spacing_y = max(float(dot_diameter_mm) * 2.5, 0.20)
    dots = [
        (cx, y_offset + j * spacing_y)
        for j in range(max(1, int(num_dots_y)))
        for cx in xs
    ]

    max_cy = max(cy for _cx, cy in dots)
    total_rows = max(1, math.ceil((max_cy + r + 0.05) / y_step_mm))

    offsets = facet_offsets_mm or {}
    rows_out: list[tuple[int, int, list[tuple[int, int]]]] = []
    r2 = r * r
    for row_idx in range(total_rows):
        facet_idx = row_idx % num_facets
        scan_mm, orth_mm = offsets.get(facet_idx, (0.0, 0.0))
        v_line = row_idx * y_step_mm + float(orth_mm)
        intervals: list[tuple[float, float]] = []
        for cx, cy in dots:
            dy = v_line - cy
            if abs(dy) < r:
                half = math.sqrt(max(0.0, r2 - dy * dy))
                intervals.append((cx - half, cx + half))
        if not intervals:
            continue
        events = _intervals_to_events(
            intervals, curve, scan_offset_mm=float(scan_mm), intensity=intensity
        )
        if events:
            rows_out.append((row_idx, facet_idx, events))

    return {
        "rows": rows_out,
        "total_rows": total_rows,
        "y_step_mm": y_step_mm,
        "num_facets": num_facets,
        "dot_centers": dots,
    }


class VirtualStackerUlab:
    """MicroPython (ulab) and NumPy 2D frame stacker replicating physical stage stepping."""

    def __init__(
        self,
        expected_total_steps: int,
        exposures: int = 4,
        px_size_um: float = 4.4,
        steps_mm: float = 100.0,
        rotation_angle_deg: float = 0.0,
    ):
        self.expected_total_steps = max(1, int(expected_total_steps))
        self.step_px = (
            (1.0 / float(steps_mm)) * 1000.0 / (float(px_size_um) * float(exposures))
        )
        angle_rad = math.radians(float(rotation_angle_deg))
        self.step_dx = -math.sin(angle_rad) * self.step_px
        self.step_dy = math.cos(angle_rad) * self.step_px
        self.canvas = None
        self.offset_x = 0
        self.offset_y = 0

    def _init_canvas(self, h: int, w: int) -> None:
        total_dx = abs(self.step_dx * self.expected_total_steps)
        total_dy = abs(self.step_dy * self.expected_total_steps)
        pad = 24
        cw = int(math.ceil(w + total_dx + 2 * pad))
        ch = int(math.ceil(h + total_dy + 2 * pad))
        self.offset_x = pad if self.step_dx >= 0 else int(math.ceil(total_dx)) + pad
        self.offset_y = pad if self.step_dy >= 0 else int(math.ceil(total_dy)) + pad
        self.canvas = np.zeros((ch, cw), dtype=np.uint8)

    def add_exposure(self, img, num_steps: int, dark_frame=None) -> None:
        clean = subtract_dark_frame_u8(img, dark_frame)
        h, w = clean.shape
        if self.canvas is None:
            self._init_canvas(h, w)

        sx = round(self.offset_x + num_steps * self.step_dx)
        sy = round(self.offset_y + num_steps * self.step_dy)
        ch, cw = self.canvas.shape

        x0 = max(0, sx)
        y0 = max(0, sy)
        x1 = min(cw, sx + w)
        y1 = min(ch, sy + h)
        if x1 <= x0 or y1 <= y0:
            return

        ix0 = x0 - sx
        iy0 = y0 - sy
        ix1 = ix0 + (x1 - x0)
        iy1 = iy0 + (y1 - y0)
        self.canvas[y0:y1, x0:x1] = np.maximum(
            self.canvas[y0:y1, x0:x1], clean[iy0:iy1, ix0:ix1]
        )

    def get_result(self):
        """Return accumulated 2D uint8 grayscale stacked canvas."""
        return self.canvas
