import logging
import math
from io import BytesIO
from pathlib import Path
from time import time

import numpy as np
from PIL import EpsImagePlugin, Image

from hexastorm.config import PlatformConfig

from . import geometry, io, plotter
from .coordinates import (
    CoordinateTransform,
    LaserheadMount,
    MachineFrame,
    SubstratePlacement,
)
from .vector import PrismCurve

# Disable PIL Decompression Bomb Warning
Image.MAX_IMAGE_PIXELS = None

logger = logging.getLogger(__name__)

DEFAULT_BITMAP_PIXELSIZE_MM = 0.3527777778  # 72 DPI fallback when metadata absent


def _resolve_bitmap_pixelsize(
    img: Image.Image, pixelsize: float | None = None
) -> float:
    """Resolve physical pixel size [mm] from argument, PIL DPI metadata, or 72-DPI fallback."""
    if pixelsize is not None:
        px = float(pixelsize)
        if px <= 0:
            raise ValueError(f"pixelsize must be positive, got {px}")
        return px
    dpi = img.info.get("dpi")
    if isinstance(dpi, (tuple, list)) and len(dpi) >= 1:
        try:
            dpi_x = float(dpi[0])
            if dpi_x > 0:
                return 25.4 / dpi_x
        except (TypeError, ValueError):
            pass
    return DEFAULT_BITMAP_PIXELSIZE_MM


def _traverse_bitmap_scanline(
    intensity_img: np.ndarray,
    oob_intensity: int,
    cx0: float,
    cy0: float,
    cx1: float,
    cy1: float,
    slope_den: float,
    curve: PrismCurve,
) -> list[tuple[int, int]]:
    """Slice one laser scanline across native bitmap pixels directly into ``24 MHz`` RLE events."""
    h_px, w_px = intensity_img.shape
    dcx = cx1 - cx0
    dcy = cy1 - cy0
    s_enter = 0.0
    s_exit = 1.0

    def _oob_events() -> list[tuple[int, int]]:
        return [(0, oob_intensity)] if (oob_intensity > 0 and curve.total_ticks > 0) else []

    if abs(dcx) < 1e-12:
        if cx0 < 0.0 or cx0 >= float(w_px):
            return _oob_events()
    else:
        sx0 = (0.0 - cx0) / dcx
        sx1 = (float(w_px) - cx0) / dcx
        if sx0 > sx1:
            sx0, sx1 = sx1, sx0
        s_enter = max(s_enter, sx0)
        s_exit = min(s_exit, sx1)

    if abs(dcy) < 1e-12:
        if cy0 < 0.0 or cy0 >= float(h_px):
            return _oob_events()
    else:
        sy0 = (0.0 - cy0) / dcy
        sy1 = (float(h_px) - cy0) / dcy
        if sy0 > sy1:
            sy0, sy1 = sy1, sy0
        s_enter = max(s_enter, sy0)
        s_exit = min(s_exit, sy1)

    if s_exit <= s_enter + 1e-12:
        return _oob_events()

    parts = [np.array([s_enter], dtype=np.float64)]
    if abs(dcx) > 1e-12 and w_px > 1:
        cx_a = cx0 + s_enter * dcx
        cx_b = cx0 + s_exit * dcx
        lo_x = min(cx_a, cx_b)
        hi_x = max(cx_a, cx_b)
        k_first = max(1, math.floor(lo_x) + 1)
        k_last = min(w_px - 1, math.ceil(hi_x) - 1)
        if k_first <= k_last:
            ks = np.arange(k_first, k_last + 1, dtype=np.float64)
            s_x = (ks - cx0) / dcx
            s_x = s_x[(s_x > s_enter + 1e-12) & (s_x < s_exit - 1e-12)]
            if s_x.size:
                parts.append(s_x)

    if abs(dcy) > 1e-12 and h_px > 1:
        cy_a = cy0 + s_enter * dcy
        cy_b = cy0 + s_exit * dcy
        lo_y = min(cy_a, cy_b)
        hi_y = max(cy_a, cy_b)
        m_first = max(1, math.floor(lo_y) + 1)
        m_last = min(h_px - 1, math.ceil(hi_y) - 1)
        if m_first <= m_last:
            ms = np.arange(m_first, m_last + 1, dtype=np.float64)
            s_y = (ms - cy0) / dcy
            s_y = s_y[(s_y > s_enter + 1e-12) & (s_y < s_exit - 1e-12)]
            if s_y.size:
                parts.append(s_y)

    parts.append(np.array([s_exit], dtype=np.float64))
    s_breaks = np.sort(np.concatenate(parts)) if len(parts) > 2 else np.concatenate(parts)

    keep = (s_breaks[1:] - s_breaks[:-1]) > 1e-12
    if not np.any(keep):
        return _oob_events()

    seg_s0 = s_breaks[:-1][keep]
    seg_s1 = s_breaks[1:][keep]
    s_mid = 0.5 * (seg_s0 + seg_s1)
    col = np.clip(np.floor(cx0 + s_mid * dcx).astype(np.int32), 0, w_px - 1)
    row = np.clip(np.floor(cy0 + s_mid * dcy).astype(np.int32), 0, h_px - 1)
    vals = intensity_img[row, col]

    if s_enter > 1e-12:
        seg_s0 = np.concatenate((np.array([0.0], dtype=np.float64), seg_s0))
        vals = np.concatenate((np.array([oob_intensity], dtype=np.uint8), vals))
    if s_exit < 1.0 - 1e-12:
        seg_s0 = np.concatenate((seg_s0, np.array([s_exit], dtype=np.float64)))
        vals = np.concatenate((vals, np.array([oob_intensity], dtype=np.uint8)))

    changes = np.flatnonzero(vals[1:] != vals[:-1]) + 1
    run_idx = (
        np.concatenate((np.array([0], dtype=np.int64), changes))
        if changes.size
        else np.array([0], dtype=np.int64)
    )
    run_s = seg_s0[run_idx]
    run_vals = vals[run_idx]

    if run_vals.size == 1 and int(run_vals[0]) == 0:
        return []

    u_base = (
        run_s * slope_den if curve.invert_hw_scan_dir else (1.0 - run_s) * slope_den
    )
    run_ticks = curve.u_to_tick(u_base)

    events: list[tuple[int, int]] = []
    cur_val = 0
    total_ticks = curve.total_ticks
    for tick_val, inten_val in zip(run_ticks.tolist(), run_vals.tolist()):
        if tick_val >= total_ticks:
            continue
        if events and events[-1][0] == tick_val:
            prev_state = events[-2][1] if len(events) >= 2 else 0
            if inten_val == prev_state:
                events.pop()
                cur_val = prev_state
            else:
                events[-1] = (tick_val, inten_val)
                cur_val = inten_val
        elif inten_val != cur_val:
            events.append((tick_val, inten_val))
            cur_val = inten_val

    return events


def quantize_grayscale(
    arr: np.ndarray,
    gray_levels: int = 32,
    intensity_lut: np.ndarray | list[int] | None = None,
) -> np.ndarray:
    """Quantize 0..255 intensities into ``gray_levels`` steps with optional 256-entry LUT."""
    levels = int(gray_levels)
    if levels < 2 or levels > 256:
        raise ValueError(f"gray_levels must be in 2..256, got {levels}")

    vals = np.clip(np.asarray(arr), 0, 255).astype(np.float64)
    idx = np.rint(vals * (levels - 1) / 255.0)
    out = np.rint(idx * (255.0 / (levels - 1))).astype(np.uint8)

    if intensity_lut is not None:
        lut = np.asarray(intensity_lut, dtype=np.uint8)
        if lut.shape != (256,):
            raise ValueError(f"intensity_lut must have length 256, got shape {lut.shape}")
        out = lut[out]
    return out


class RasterExposure:
    """
    Raster exposure & fallback pipeline for converting bitmaps and rasterized vectors into FPGA streams.

    * **True bitmaps (``.png``, ``.jpg``, ``.bmp``, ``.tif``, ...):** Keep their native pixel
      dimensions (no resizing onto ``samplegridsize``) and are traversed directly onto
      ``24 MHz`` FPGA clock-tick RLE events via :meth:`slice_bitmap` and :class:`PrismCurve`.
    * **Vector-to-raster fallback (``.ps``/``.eps`` and explicitly rasterized ``.svg``):**
      Use ``samplegridsize`` (default ``0.01 mm`` = ``10 µm``) as the software rasterization pitch
      via the golden-reference inverse-mapping path (:meth:`patternfile`).
    """

    def __init__(
        self,
        correction: bool | dict = False,
        exposures: int = 1,
        mount: LaserheadMount | None = None,
        placement: SubstratePlacement | None = None,
        machine: MachineFrame | None = None,
        grayscale: bool = False,
        gray_levels: int = 32,
        intensity_lut: np.ndarray | list[int] | None = None,
    ):
        self.cfg = PlatformConfig(test=False)
        self.correction = correction
        self.exposures = exposures
        self.mount = mount if mount is not None else LaserheadMount()
        self.placement = placement if placement is not None else SubstratePlacement()
        self.machine = (
            machine if machine is not None else MachineFrame.from_config(self.cfg)
        )
        self.grayscale = bool(grayscale)
        self.gray_levels = int(gray_levels)
        self.intensity_lut = intensity_lut
        self._last_coord: CoordinateTransform | None = None
        self._last_active_lines: int = 0
        self.set_optical_params(correction=correction, exposures=exposures)
        self.debug_folder = self.cfg.paths["base"] / "debug"
        self.debug_folder.mkdir(parents=True, exist_ok=True)
        self._last_output_path = None  # To track the last output pat file for reading
        self.bitorder = "big"

    def set_optical_params(
        self, correction: bool | dict | None = None, exposures: int | None = None
    ):
        if correction is not None:
            self.correction = correction
        if exposures is not None:
            self.exposures = exposures
        raw_params = self.cfg.get_optical_params(
            correction=self.correction, exposures=self.exposures
        )
        self.geo = geometry.ScannerModel(raw_params)
        self.params = self.geo.params
        self.curve = PrismCurve(
            self.params, invert_hw_scan_dir=self.mount.invert_hw_scan_dir
        )

    def svgtopil(self, svg_filepath: Path) -> Image.Image:
        """Convert SVG vector graphics to an unrotated RGB PIL Image using resvg-py."""
        import resvg_py

        dpi = float(round(25.4 / self.params["samplegridsize"]))
        png_data = resvg_py.svg_to_bytes(
            svg_string=Path(svg_filepath).read_text(encoding="utf-8"),
            dpi=dpi,
            background="#ffffff",
        )
        img = Image.open(BytesIO(png_data))
        if img.mode in ("RGBA", "LA"):
            bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
            img = Image.alpha_composite(bg, img.convert("RGBA")).convert("RGB")
        return img

    def pstopil(
        self, filepath: Path, pixelsize: float = DEFAULT_BITMAP_PIXELSIZE_MM
    ) -> Image.Image:
        """Rasterize PostScript/EPS vectors using ``samplegridsize`` as fallback resolution."""
        img = Image.open(filepath)
        grid_size = float(self.params["samplegridsize"])
        scale_int = max(1, round(float(pixelsize) / grid_size))
        if isinstance(img, EpsImagePlugin.EpsImageFile):
            img.load(scale=scale_int)
            img.info["_hexastorm_pixel_mm"] = float(pixelsize) / scale_int
        else:
            img.load()
            img.info["_hexastorm_pixel_mm"] = _resolve_bitmap_pixelsize(img, pixelsize)
        return img

    def imgtopil(
        self, filepath: Path, pixelsize: float | None = None
    ) -> Image.Image:
        """Load a bitmap image at its native pixel dimensions without re-discretizing."""
        img = Image.open(filepath)
        img.load()
        dpi = img.info.get("dpi")
        if img.mode in ("RGBA", "LA") or (
            img.mode == "P" and "transparency" in img.info
        ):
            bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
            img = Image.alpha_composite(bg, img.convert("RGBA")).convert("RGB")
            if dpi is not None:
                img.info["dpi"] = dpi
        return img

    def piltoarray(
        self,
        img: Image.Image,
        grayscale: bool | None = None,
        pixel_size_mm: float | None = None,
    ) -> np.ndarray:
        """Convert PIL Image (unrotated Substrate frame) to binary or 8-bit grayscale array."""
        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        if use_gray:
            img_array = np.array(img.convert("L"), dtype=np.uint8)
        else:
            img_array = np.array(img.convert("1", dither=Image.Dither.NONE))
        if img_array.max() == 0:
            raise ValueError("Image is empty")

        px_mm = (
            float(self.params["samplegridsize"])
            if pixel_size_mm is None
            else float(pixel_size_mm)
        )
        if px_mm <= 0:
            raise ValueError(f"pixel_size_mm must be positive, got {px_mm}")

        h_px, w_px = img_array.shape
        width_mm = w_px * px_mm
        height_mm = h_px * px_mm

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

        logger.debug(
            f"Sample size is {coord.u_size_mm:.2f} mm by {coord.v_size_mm:.2f} mm"
        )
        self.params["samplexsize"] = coord.u_size_mm
        self.params["sampleysize"] = coord.v_size_mm
        return img_array

    def laser_compensation(
        self,
        layerarr: np.ndarray,
        erode: bool,
        pixel_size_mm: float | None = None,
    ) -> np.ndarray:
        """Elliptical laser spot morphology compensation."""
        return plotter.laser_compensation(
            layerarr, self.params, erode=erode, pixel_size_mm=pixel_size_mm
        )

    def _resolve_input_path(self, url: str | Path) -> Path:
        file_path = Path(url)
        if not file_path.is_absolute() and not file_path.exists():
            svg_candidate = self.cfg.paths["svgs"] / file_path
            img_candidate = self.cfg.paths["images"] / file_path
            if svg_candidate.exists():
                return svg_candidate
            if img_candidate.exists():
                return img_candidate
            return svg_candidate
        return file_path

    def _prepare_source_array(
        self,
        file_path: Path,
        pixelsize: float | None = None,
        test: bool = False,
        laser_compensate: bool = False,
        use_gray: bool = False,
    ) -> tuple[np.ndarray, float, str]:
        """Load image/vector fallback at appropriate native or fallback pixel pitch."""
        suffix = file_path.suffix.lower()
        grid_size = float(self.params["samplegridsize"])
        if suffix == ".svg":
            pil = self.svgtopil(file_path)
            if test:
                self.debug_folder.mkdir(parents=True, exist_ok=True)
                pil.save(self.debug_folder / "read_debug.png")
            eff_px_mm = grid_size
        elif suffix in (".ps", ".eps"):
            pil = self.pstopil(
                file_path,
                pixelsize=DEFAULT_BITMAP_PIXELSIZE_MM if pixelsize is None else pixelsize,
            )
            eff_px_mm = float(pil.info.get("_hexastorm_pixel_mm", grid_size))
        else:
            pil = self.imgtopil(file_path, pixelsize)
            eff_px_mm = _resolve_bitmap_pixelsize(pil, pixelsize)

        layerarr = self.piltoarray(
            pil, grayscale=use_gray, pixel_size_mm=eff_px_mm
        ).astype(np.uint8)
        if laser_compensate and not use_gray:
            layerarr = self.laser_compensation(
                layerarr, erode=False, pixel_size_mm=eff_px_mm
            )
            self.debug_folder.mkdir(parents=True, exist_ok=True)
            Image.fromarray(layerarr.astype(np.uint8) * 255).save(
                self.debug_folder / "erosioncheck.png"
            )
        if test:
            self.debug_folder.mkdir(parents=True, exist_ok=True)
            vis = layerarr if use_gray else layerarr.astype(np.uint8) * 255
            Image.fromarray(vis).save(self.debug_folder / "simplecheck.png")
        return layerarr, eff_px_mm, suffix

    def slice_bitmap(
        self,
        layerarr: np.ndarray,
        pixel_size_mm: float,
        filepath: str | Path,
        grayscale: bool | None = None,
        gray_levels: int | None = None,
        intensity_lut: np.ndarray | list[int] | None = None,
        positiveresist: bool = False,
        compression_level: int = 9,
    ) -> Path:
        """Slice a native bitmap array directly into ``24 MHz`` FPGA clock-tick RLE events."""
        arr = np.asarray(layerarr)
        if arr.ndim != 2 or arr.size == 0:
            raise ValueError("Expected non-empty 2D bitmap array")
        px_mm = float(pixel_size_mm)
        if px_mm <= 0:
            raise ValueError(f"pixel_size_mm must be positive, got {px_mm}")

        h_px, w_px = arr.shape
        width_mm = w_px * px_mm
        height_mm = h_px * px_mm

        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        levels = self.gray_levels if gray_levels is None else int(gray_levels)
        lut = self.intensity_lut if intensity_lut is None else intensity_lut
        use_pos = bool(positiveresist or self.params["positiveresist"])

        if use_gray:
            raw_u8 = arr.astype(np.uint8) if use_pos else (255 - arr.astype(np.uint8))
            intensity_img = quantize_grayscale(raw_u8, levels, lut)
            oob_raw = np.array([255 if use_pos else 0], dtype=np.uint8)
            oob_intensity = int(quantize_grayscale(oob_raw, levels, lut)[0])
        else:
            is_white = arr > 0
            if use_pos:
                intensity_img = np.where(is_white, 255, 0).astype(np.uint8)
                oob_intensity = 255
            else:
                intensity_img = np.where(is_white, 0, 255).astype(np.uint8)
                oob_intensity = 0

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

        params = self.params
        u_size_mm, v_size_mm = coord.u_size_mm, coord.v_size_mm
        params["samplexsize"] = float(u_size_mm)
        params["sampleysize"] = float(v_size_mm)

        curve = self.curve
        if curve.invert_hw_scan_dir != self.mount.invert_hw_scan_dir:
            curve = PrismCurve(
                params, invert_hw_scan_dir=self.mount.invert_hw_scan_dir
            )
            self.curve = curve

        lanewidth = curve.lanewidth
        facets = int(params["facets"])
        stagespeed = float(params["stagespeed"])
        rot_freq = float(params["rotationfrequency"])
        stepsperline = float(params["stepsperline"])

        lanes = math.ceil(u_size_mm / lanewidth)
        facets_inlane = math.ceil(rot_freq * facets * (v_size_mm / stagespeed))
        params["lanewidth"] = float(lanewidth)
        params["lanes"] = float(lanes)
        params["facetsinlane"] = float(facets_inlane)

        facet_period_s = 1.0 / (rot_freq * facets)
        y_shift_per_facet = stagespeed * facet_period_s
        v_prism_start = -curve.cos_tilt * curve.disp_last
        v_prism_end = -curve.cos_tilt * curve.disp_first
        v_stage_start = (curve.end_frac * facet_period_s) * stagespeed
        v_stage_end = (curve.start_frac * facet_period_s) * stagespeed
        slope_den = curve.raw_u_first - curve.raw_u_last

        dx_facets = [float(params[f"f{fi}_scan"]) for fi in range(facets)]
        dy_facets = [float(params[f"f{fi}_orth"]) for fi in range(facets)]

        # Map Laserhead (u, v) [mm] -> continuous PIL pixel coords (cx, cy) with y=height_mm at row 0
        m = coord.lh_to_sub_matrix
        inv_px = 1.0 / px_mm
        c_u = float(m[0, 0]) * inv_px
        c_v = float(m[0, 1]) * inv_px
        c_0 = float(m[0, 2]) * inv_px
        r_u = -float(m[1, 0]) * inv_px
        r_v = -float(m[1, 1]) * inv_px
        r_0 = (height_mm - float(m[1, 2])) * inv_px

        out_path = Path(filepath)
        self._last_output_path = out_path
        active_lines = 0
        u_origin_base = curve.raw_u_last + curve.xstart_val

        logger.info(f"Writing native bitmap event stream to {out_path}...")
        with io.EventStreamWriter(
            filepath=out_path,
            lanewidth=lanewidth,
            facets_in_lane=facets_inlane,
            lanes=lanes,
            steps_per_line=stepsperline,
            scanline_ticks=curve.total_ticks,
            compression_level=compression_level,
        ) as writer:
            for lane_idx in range(lanes):
                direction = 1 if lane_idx % 2 == 0 else 0
                is_forward = lane_idx % 2 == 0
                dir_sign = 1.0 if is_forward else -1.0
                lane_u_offset = lane_idx * lanewidth
                lane_v_stage_start = dir_sign * v_stage_start
                lane_v_stage_end = dir_sign * v_stage_end

                for f_lane_idx in range(facets_inlane):
                    facet_idx = f_lane_idx % facets
                    dx_facet = dx_facets[facet_idx]
                    dy_facet = dy_facets[facet_idx]
                    ystart_mm = (
                        f_lane_idx * y_shift_per_facet
                        if is_forward
                        else (facets_inlane - f_lane_idx) * y_shift_per_facet
                    )

                    u_start = u_origin_base + dx_facet + lane_u_offset
                    u_end = u_start + slope_den
                    v_start = v_prism_start + dy_facet + lane_v_stage_start + ystart_mm
                    v_end = v_prism_end + dy_facet + lane_v_stage_end + ystart_mm

                    if curve.invert_hw_scan_dir:
                        u0, v0, u1, v1 = u_start, v_start, u_end, v_end
                    else:
                        u0, v0, u1, v1 = u_end, v_end, u_start, v_start

                    cx0 = c_u * u0 + c_v * v0 + c_0
                    cy0 = r_u * u0 + r_v * v0 + r_0
                    cx1 = c_u * u1 + c_v * v1 + c_0
                    cy1 = r_u * u1 + r_v * v1 + r_0

                    events = _traverse_bitmap_scanline(
                        intensity_img,
                        oob_intensity,
                        cx0,
                        cy0,
                        cx1,
                        cy1,
                        slope_den,
                        curve,
                    )
                    if events:
                        active_lines += 1
                    writer.write_scanline(events, direction)

        self._last_active_lines = active_lines
        return out_path

    def patternfile(
        self,
        url: str | Path,
        pixelsize: float | None = None,
        test: bool = False,
        positiveresist: bool = False,
        laser_compensate: bool = False,  # remove laser spot size
        radius_mm: float = 0.015,  # 15um radius = 30um diameter
        grayscale: bool | None = None,
        gray_levels: int | None = None,
        intensity_lut: np.ndarray | list[int] | None = None,
    ) -> np.ndarray:
        """Sample an image or rasterized vector onto the ``samplegridsize`` reference grid."""
        ctime = time()
        file_path = self._resolve_input_path(url)
        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        levels = self.gray_levels if gray_levels is None else int(gray_levels)
        lut = self.intensity_lut if intensity_lut is None else intensity_lut
        use_pos = bool(positiveresist or self.params["positiveresist"])

        layerarr, eff_px_mm, _suffix = self._prepare_source_array(
            file_path,
            pixelsize=pixelsize,
            test=test,
            laser_compensate=laser_compensate,
            use_gray=use_gray,
        )

        logger.debug("Retrieved image")
        logger.debug(f"Elapsed {time() - ctime:.2f} seconds")

        # Map Laserhead Coordinates -> Substrate Bitmap via CoordinateTransform
        coord = self._last_coord
        h_px, w_px = layerarr.shape
        grid_size = float(self.params["samplegridsize"])

        if (coord is None or coord.is_identity_lh_to_sub) and math.isclose(
            eff_px_mm, grid_size, rel_tol=1e-9, abs_tol=1e-12
        ):
            ids = self.geo.calculate_coordinates()
            logger.debug("Created coordinates for interpolation")
            logger.debug(f"Elapsed {time() - ctime:.2f} seconds")

            u_px = ids[0]
            v_px = ids[1]
            mask = (u_px >= 0) & (u_px < w_px) & (v_px >= 0) & (v_px < h_px)
            ptrn = np.zeros(u_px.shape, dtype=np.uint8)
            # Substrate y=0 is bottom row (h_px - 1), y=height_mm is top row (0)
            ptrn[mask] = layerarr[(h_px - 1) - v_px[mask], u_px[mask]]
        else:
            ptrn_3d: np.ndarray | None = None
            mask_3d: np.ndarray | None = None
            for lane_idx, u_mm, v_mm in self.geo.iter_lane_coords_mm():
                if ptrn_3d is None:
                    n_lanes = int(self.params["lanes"])
                    n_facets = int(self.params["facetsinlane"])
                    n_pts = u_mm.shape[1]
                    ptrn_3d = np.zeros((n_lanes, n_facets, n_pts), dtype=np.uint8)
                    mask_3d = np.zeros((n_lanes, n_facets, n_pts), dtype=bool)
                    logger.debug(f"The lanewidth is {self.params['lanewidth']:.2f} mm")
                    logger.debug(f"The facets in lane are {n_facets}")

                if coord is None or coord.is_identity_lh_to_sub:
                    x_sub_mm, y_sub_mm = u_mm, v_mm
                    box_w = w_px * eff_px_mm
                    box_h = h_px * eff_px_mm
                else:
                    x_sub_mm, y_sub_mm = coord.laserhead_to_substrate_xy(u_mm, v_mm)
                    box_w = coord.width_mm
                    box_h = coord.height_mm

                in_bounds = (
                    (x_sub_mm >= 0.0)
                    & (x_sub_mm <= box_w)
                    & (y_sub_mm >= 0.0)
                    & (y_sub_mm <= box_h)
                )
                col = np.clip(
                    np.floor(x_sub_mm[in_bounds] / eff_px_mm).astype(np.int32),
                    0,
                    w_px - 1,
                )
                row = np.clip(
                    np.floor(
                        (box_h - y_sub_mm[in_bounds]) / eff_px_mm
                    ).astype(np.int32),
                    0,
                    h_px - 1,
                )
                ptrn_3d[lane_idx][in_bounds] = layerarr[row, col]
                mask_3d[lane_idx] = in_bounds

            assert ptrn_3d is not None and mask_3d is not None
            ptrn = ptrn_3d.ravel()
            mask = mask_3d.ravel()

        logger.debug("Completed interpolation")
        logger.debug(f"Elapsed {time() - ctime:.2f} seconds")

        if use_gray:
            if not use_pos:
                ptrn = (255 - ptrn).astype(np.uint8)
                ptrn[~mask] = 0
            else:
                ptrn[~mask] = 255
            ptrn = quantize_grayscale(ptrn, levels, lut)
            ptrn = np.repeat(ptrn, self.params["downsamplefactor"]).astype(np.uint8)
            return ptrn

        if ptrn.min() < 0 or ptrn.max() > 1:
            raise ValueError("This is not a bit list")

        # Handle Polarity and Padding ('mask' = valid area, '~mask' = overscan)
        if not use_pos:
            ptrn = np.logical_not(ptrn)
            ptrn[~mask] = 0
        else:
            ptrn[~mask] = 1

        ptrn = np.repeat(ptrn, self.params["downsamplefactor"])
        ptrn = np.packbits(ptrn, bitorder=self.bitorder)
        return ptrn

    def _machine_job_duration(self) -> float:
        laz_tim = self.cfg.laser_timing
        lanes = self.params["lanes"]
        facets = self.params["facetsinlane"]
        line_min = laz_tim["facets"] * laz_tim["rpm"]
        expose_time = (lanes * facets) / line_min
        return 0.8 + expose_time + lanes * 0.2

    def img_to_bin(
        self,
        img_name: str | Path,
        bin_name: str | Path | None = None,
        camera: bool = False,
        **kwargs,
    ) -> float:
        """Convert bitmap (direct native ``pixelsize`` slicing) or vector fallback to ``.pat`` and return minutes."""
        img_path_arg = Path(img_name)
        file_path = self._resolve_input_path(img_path_arg)
        out_target = (
            img_path_arg.with_suffix(".pat") if not bin_name else Path(bin_name)
        )
        if not out_target.is_absolute():
            out_target = self.cfg.paths["patterns"] / out_target

        use_gray = kwargs.get("grayscale")
        if use_gray is None:
            use_gray = self.grayscale

        if file_path.suffix.lower() in (".svg", ".ps", ".eps"):
            ptrn = self.patternfile(file_path, **kwargs)
            self.writebin(ptrn, out_target, grayscale=bool(use_gray))
            duration_min = self.estimate_job_duration(
                ptrn, camera=camera, grayscale=bool(use_gray)
            )
        else:
            layerarr, eff_px_mm, _suffix = self._prepare_source_array(
                file_path,
                pixelsize=kwargs.get("pixelsize"),
                test=bool(kwargs.get("test", False)),
                laser_compensate=bool(kwargs.get("laser_compensate", False)),
                use_gray=bool(use_gray),
            )
            tagged_out = self._tagged_bin_path(out_target)
            self.slice_bitmap(
                layerarr=layerarr,
                pixel_size_mm=eff_px_mm,
                filepath=tagged_out,
                grayscale=bool(use_gray),
                gray_levels=kwargs.get("gray_levels"),
                intensity_lut=kwargs.get("intensity_lut"),
                positiveresist=bool(kwargs.get("positiveresist", False)),
            )
            duration_min = (
                self._last_active_lines / 180.0
                if camera
                else self._machine_job_duration()
            )

        device = "camera" if camera else "machine"
        logger.info(
            f"Estimated Job Duration: {duration_min:.2f} minutes using {device}"
        )
        return duration_min

    def estimate_job_duration(
        self,
        ptrn_packed: np.ndarray,
        camera: bool = False,
        grayscale: bool | None = None,
    ) -> float:
        """
        Calculates expected job duration in minutes directly from the pattern array.
        """
        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        if camera:
            bits_in_scanline = int(self.params["bitsinscanline"])
            if use_gray:
                lines2d = np.asarray(ptrn_packed, dtype=np.uint8).reshape(
                    -1, bits_in_scanline
                )
            else:
                data_bits = np.unpackbits(ptrn_packed, bitorder=self.bitorder)
                lines2d = data_bits.reshape(-1, bits_in_scanline)

            # Count rows where the sum is greater than 0
            active_lines = np.count_nonzero(lines2d.sum(axis=1) > 0)

            # 3 lines per second = 180 lines per minute
            return active_lines / 180.0

        return self._machine_job_duration()

    def _tagged_bin_path(self, path: Path) -> Path:
        """Appends exposure count and optical-correction tag to the output stem."""
        return io.tagged_bin_path(path, self.params)

    def writebin(
        self,
        pixeldata: np.ndarray,
        filename: str | Path = "test.bin",
        grayscale: bool | None = None,
    ):
        """Wrapper for ``io.write_binary_file`` (or ``io.write_grayscale_binary_file``)."""
        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        out_path = Path(filename)
        if not out_path.is_absolute():
            out_path = self.cfg.paths["patterns"] / out_path

        out_path = self._tagged_bin_path(out_path)

        self._last_output_path = out_path
        writer_fn = (
            io.write_grayscale_binary_file if use_gray else io.write_binary_file
        )
        writer_fn(
            pixeldata,
            self.params,
            out_path,
            invert_hw_scan_dir=self.mount.invert_hw_scan_dir,
        )

    def readbin(
        self, filename: str | Path | None = None, grayscale: bool | None = None
    ) -> dict:
        """Wrapper for ``io.read_binary_file`` (or ``io.read_grayscale_binary_file``)."""
        use_gray = self.grayscale if grayscale is None else bool(grayscale)
        if filename is None:
            filename = self._last_output_path
        in_path = Path(filename)
        if not in_path.is_absolute():
            in_path = self.cfg.paths["patterns"] / in_path
        if not in_path.exists():
            tagged_path = self._tagged_bin_path(in_path)
            if tagged_path.exists():
                in_path = tagged_path

        if use_gray:
            facets, lanes, width, data = io.read_grayscale_binary_file(
                in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
            )
        else:
            facets, lanes, width, data_bytes = io.read_binary_file(
                in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
            )
            data = np.unpackbits(data_bytes, bitorder=self.bitorder)

        # Return a clean dictionary
        return {
            "data": data,
            "metadata": {
                "lanes": lanes,
                "lanewidth": width,
                "facetsinlane": facets,
            },
        }

    def plotptrn(
        self,
        ptrn_data: np.ndarray | dict,
        filename: str = "plot",
        color_lanes: bool = True,
        view: str = "substrate",
    ) -> np.ndarray:
        """Visualizes the generated pattern data via :mod:`hexastorm.exposure.plotter`."""
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
