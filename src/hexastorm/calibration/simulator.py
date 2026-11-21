"""First-order perturbation optical simulator ("Fake Scanhead") for closed-loop testing.

Models a physical polygon laser scanner with deterministic first-order mechanical
facet offsets (along-scan phase offset + orthogonal transverse offset), Snell's-law
beam velocity curve, 1:3 elliptical or circular focused spots, camera tilt, and
optional white sensor noise without requiring physical hardware or OpenCV.
"""

from __future__ import annotations

import math

import numpy as np

from .. import ulabext
from ..config import PlatformConfig
from .core import PrismCurveLite


class FakeScanhead:
    """First-order optical forward simulator mapping 24 MHz FPGA events onto a camera sensor."""

    def __init__(
        self,
        params: dict | None = None,
        facet_offsets_um: dict[int, tuple[float, float]] | None = None,
        rotation_deg: float = 1.5,
        pixel_size_um: float = 4.4,
        sensor_shape: tuple[int, int] = (600, 800),
        u_center_mm: float | None = None,
        v_center_mm: float = 0.0,
        spot_sigma_scan_um: float = 10.0,
        spot_sigma_orth_um: float = 28.0,
        dark_level: int = 15,
        peak_intensity: float = 230.0,
        noise_std: float = 0.0,
        seed: int = 42,
        invert_hw_scan_dir: bool = True,
    ):
        if params is None:
            params = PlatformConfig(test=False).get_optical_params(
                correction=False, exposures=1
            )
        self.params = dict(params)
        self.curve = PrismCurveLite(
            self.params, invert_hw_scan_dir=bool(invert_hw_scan_dir)
        )
        default_offsets = {
            0: (14.0, -9.0),
            1: (-44.0, 36.0),
            2: (26.0, -24.0),
            3: (-12.0, 13.0),
        }
        src_offsets = default_offsets if facet_offsets_um is None else facet_offsets_um
        self.facet_offsets_um: dict[int, tuple[float, float]] = {
            int(k): (float(v[0]), float(v[1])) for k, v in src_offsets.items()
        }
        self.rotation_deg = float(rotation_deg)
        self.pixel_size_um = float(pixel_size_um)
        self.sensor_shape = (int(sensor_shape[0]), int(sensor_shape[1]))
        self.u_center_mm = (
            self.curve.lanewidth * 0.5 if u_center_mm is None else float(u_center_mm)
        )
        self.v_center_mm = float(v_center_mm)
        self.spot_sigma_scan_um = float(spot_sigma_scan_um)
        self.spot_sigma_orth_um = float(spot_sigma_orth_um)
        self.dark_level = int(dark_level)
        self.peak_intensity = float(peak_intensity)
        self.noise_std = float(noise_std)
        self._rng = np.random.default_rng(seed)

    def _apply_noise_and_clip(self, canvas: np.ndarray) -> np.ndarray:
        if self.noise_std > 0.0:
            canvas = canvas + self._rng.normal(0.0, self.noise_std, size=canvas.shape)
        return np.clip(np.rint(canvas), 0, 255).astype(np.uint8)

    def render_dark_frame(self) -> np.ndarray:
        """Render a background camera frame with the laser disabled."""
        base = np.full(self.sensor_shape, float(self.dark_level), dtype=np.float64)
        return self._apply_noise_and_clip(base)

    def render_events(
        self,
        events: list[tuple[int, int]],
        facet: int = 0,
        v_stage_mm: float = 0.0,
    ) -> np.ndarray:
        """Render one facet's 24 MHz ``(tick, intensity)`` events onto the tilted sensor."""
        h, w = self.sensor_shape
        spot_acc = np.zeros((h, w), dtype=np.float64)
        if not events:
            return self._apply_noise_and_clip(spot_acc + float(self.dark_level))

        n_facets = int(self.params["facets"])
        f_idx = int(facet) % n_facets
        err_scan_um, err_orth_um = self.facet_offsets_um.get(f_idx, (0.0, 0.0))
        err_scan_mm = err_scan_um / 1000.0
        err_orth_mm = err_orth_um / 1000.0

        total_ticks = self.curve.total_ticks
        valid = sorted(
            (int(t), max(0, min(255, int(v))))
            for t, v in events
            if 0 <= int(t) < ulabext.SENTINEL_TICK
        )
        segments: list[tuple[int, int, int]] = []
        for idx, (t_on, intensity) in enumerate(valid):
            if intensity <= 0 or t_on >= total_ticks:
                continue
            t_off = valid[idx + 1][0] if idx + 1 < len(valid) else total_ticks
            t_off = min(total_ticks, max(t_on, t_off))
            if t_off > t_on:
                segments.append((t_on, t_off, intensity))

        if not segments:
            return self._apply_noise_and_clip(spot_acc + float(self.dark_level))

        sig_scan_px = max(0.5, self.spot_sigma_scan_um / self.pixel_size_um)
        sig_orth_px = max(0.5, self.spot_sigma_orth_um / self.pixel_size_um)
        inv_s2 = 1.0 / (sig_scan_px * sig_scan_px)
        inv_o2 = 1.0 / (sig_orth_px * sig_orth_px)

        theta = math.radians(self.rotation_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        inv_px_per_mm = 1000.0 / self.pixel_size_um

        r_s = 3.5 * sig_scan_px
        r_o = 3.5 * sig_orth_px
        half_x = math.ceil(abs(cos_t) * r_s + abs(sin_t) * r_o)
        half_y = math.ceil(abs(sin_t) * r_s + abs(cos_t) * r_o)

        v_phys_mm = float(v_stage_mm) + err_orth_mm
        ry_rel = (v_phys_mm - self.v_center_mm) * inv_px_per_mm
        step_u_mm = max(0.002, 0.5 * (self.spot_sigma_scan_um / 1000.0))
        half_w_px = 0.5 * w
        half_h_px = 0.5 * h

        for t_on, t_off, intensity in segments:
            ua = self.curve.tick_to_u(t_on) + err_scan_mm
            ub = self.curve.tick_to_u(t_off) + err_scan_mm
            if ub < ua:
                ua, ub = ub, ua
            span_u = ub - ua
            n_sub = max(1, math.ceil(span_u / step_u_mm))
            amp = self.peak_intensity * (intensity / 255.0)
            seg_acc = np.zeros((h, w), dtype=np.float64)

            for k in range(n_sub):
                u_s = ua + ((k + 0.5) / n_sub) * span_u if span_u > 0 else ua
                rx_rel = (u_s - self.u_center_mm) * inv_px_per_mm
                cx = half_w_px + rx_rel * cos_t - ry_rel * sin_t
                cy = half_h_px + rx_rel * sin_t + ry_rel * cos_t

                x0 = max(0, math.floor(cx - half_x))
                x1 = min(w, math.ceil(cx + half_x) + 1)
                y0 = max(0, math.floor(cy - half_y))
                y1 = min(h, math.ceil(cy + half_y) + 1)
                if x1 <= x0 or y1 <= y0:
                    continue

                xs = np.arange(x0, x1, dtype=np.float64) - cx
                ys = np.arange(y0, y1, dtype=np.float64) - cy
                u_p = xs[None, :] * cos_t + ys[:, None] * sin_t
                v_p = -xs[None, :] * sin_t + ys[:, None] * cos_t
                patch = amp * np.exp(-0.5 * ((u_p * u_p) * inv_s2 + (v_p * v_p) * inv_o2))
                seg_acc[y0:y1, x0:x1] = np.maximum(seg_acc[y0:y1, x0:x1], patch)

            spot_acc = np.maximum(spot_acc, seg_acc)

        return self._apply_noise_and_clip(spot_acc + float(self.dark_level))

    def render_bits(
        self,
        bit_list: list[int] | np.ndarray,
        facet: int = 0,
        v_stage_mm: float = 0.0,
    ) -> np.ndarray:
        """Convert a 1D binary scanline list into 24 MHz events and render it onto the sensor."""
        events = ulabext.bits_to_events(bit_list, self.curve.total_ticks)
        return self.render_events(events, facet=facet, v_stage_mm=v_stage_mm)
