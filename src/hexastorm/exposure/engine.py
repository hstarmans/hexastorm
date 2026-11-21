import logging
import math
from pathlib import Path

import fire
import numpy as np

from hexastorm.config import PlatformConfig, displacement_at_frac

from . import io, plotter
from .coordinates import (
    CoordinateTransform,
    LaserheadMount,
    MachineFrame,
    SubstratePlacement,
)
from .raster import RasterExposure
from .vector import VectorExposure

logger = logging.getLogger(__name__)

VALID_MODES = ("auto", "vector", "raster")


class ExposureEngine:
    """Unified façade and dispatcher for converting patterns into FPGA exposure files.

    Routes vector designs (``.svg``) to :class:`VectorExposure`, true bitmaps
    (``.png``, ``.jpg``, etc.) to native-resolution scanline slicing via
    :meth:`RasterExposure.slice_bitmap`, and PostScript / explicitly rasterized vectors
    to the ``samplegridsize`` fallback in :class:`RasterExposure`.
    """

    def __init__(
        self,
        correction: bool | dict = False,
        exposures: int = 1,
        mode: str = "auto",
        mount: LaserheadMount | None = None,
        intensity: int = 255,
        placement: SubstratePlacement | None = None,
        machine: MachineFrame | None = None,
        grayscale: bool = False,
        gray_levels: int = 32,
        intensity_lut: np.ndarray | list[int] | None = None,
    ):
        if mode not in VALID_MODES:
            raise ValueError(
                f"Unsupported exposure mode {mode!r}; choose from {VALID_MODES}"
            )

        self.cfg = PlatformConfig(test=False)
        self.mode = mode
        self.mount = mount
        self.placement = placement
        self.machine = (
            machine if machine is not None else MachineFrame.from_config(self.cfg)
        )
        self.default_intensity = max(1, min(255, int(intensity)))
        self.grayscale = bool(grayscale)
        self.gray_levels = int(gray_levels)
        self.intensity_lut = intensity_lut
        self.bitorder = "big"
        self._vector: VectorExposure | None = None
        self._raster: RasterExposure | None = None
        self._last_output_path: Path | None = None
        self._last_mode: str | None = None
        self._last_coord: CoordinateTransform | None = None
        self.set_optical_params(correction=correction, exposures=exposures)

    @property
    def vector(self) -> VectorExposure:
        """Lazily instantiated vector exposure backend."""
        if self._vector is None:
            self._vector = VectorExposure(
                correction=self.correction,
                exposures=self.exposures,
                mount=self.mount,
                intensity=self.default_intensity,
                placement=self.placement,
                machine=self.machine,
            )
        return self._vector

    @property
    def raster(self) -> RasterExposure:
        """Lazily instantiated raster reference backend."""
        if self._raster is None:
            self._raster = RasterExposure(
                correction=self.correction,
                exposures=self.exposures,
                mount=self.mount,
                placement=self.placement,
                machine=self.machine,
                grayscale=self.grayscale,
                gray_levels=self.gray_levels,
                intensity_lut=self.intensity_lut,
            )
        return self._raster

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
        if self._vector is not None:
            self._vector.set_optical_params(
                correction=self.correction, exposures=self.exposures
            )
        if self._raster is not None:
            self._raster.set_optical_params(
                correction=self.correction, exposures=self.exposures
            )

    def px_to_mm(self, px: int) -> float:
        """Map a software scanline pixel index to physical scan-axis distance in mm."""
        start_frac = float(self.params["start_frac"])
        end_frac = float(self.params["end_frac"])
        bits_len = int(self.params["bitsinscanline"])
        denom = max(1.0, float(bits_len - 1))
        frac = start_frac + ((px % bits_len) / denom) * (end_frac - start_frac)
        sin_tilt = math.sin(float(self.params["tiltangle"]))
        start_u = sin_tilt * displacement_at_frac(start_frac, self.params)
        return float(start_u - (sin_tilt * displacement_at_frac(frac, self.params)))

    def resolve_mode(self, input_path: str | Path, mode: str | None = None) -> str:
        """Determine which backend ('vector' or 'raster') should process *input_path*."""
        effective = self.mode if mode is None else mode
        if effective not in VALID_MODES:
            raise ValueError(
                f"Unsupported exposure mode {effective!r}; choose from {VALID_MODES}"
            )
        suffix = Path(input_path).suffix.lower()
        is_svg = suffix == ".svg"
        if effective == "auto":
            if self.grayscale:
                return "raster"
            return "vector" if is_svg else "raster"
        if effective == "vector" and not is_svg:
            raise ValueError(
                f"Vector exposure requires an .svg input, got {suffix or '<none>'}"
            )
        return effective

    def generate(
        self,
        input_path: str | Path,
        output_path: str | Path | None = None,
        camera: bool = False,
        mode: str | None = None,
        **kwargs,
    ) -> float:
        """Compile *input_path* to an event-driven ``.pat``/``.bin`` file and return duration in minutes."""
        resolved = self.resolve_mode(input_path, mode=mode)
        self._last_mode = resolved
        backend = self.vector if resolved == "vector" else self.raster
        duration = backend.img_to_bin(
            input_path, bin_name=output_path, camera=camera, **kwargs
        )
        self._last_output_path = backend._last_output_path
        self._last_coord = getattr(backend, "_last_coord", None)
        for key in ("lanes", "facetsinlane", "samplexsize", "sampleysize"):
            if key in backend.params:
                self.params[key] = backend.params[key]
        return duration

    def img_to_bin(
        self,
        img_name: str | Path,
        bin_name: str | Path | None = None,
        camera: bool = False,
        mode: str | None = None,
        **kwargs,
    ) -> float:
        """Alias for :meth:`generate`."""
        return self.generate(
            img_name, output_path=bin_name, camera=camera, mode=mode, **kwargs
        )

    def writebin(
        self, pixeldata: np.ndarray, filename: str | Path = "test.bin"
    ) -> Path:
        """Encode packed bitmap scanline data directly into an event-driven ``.bin``/``.pat`` file."""
        out_path = Path(filename)
        if not out_path.is_absolute():
            out_path = self.cfg.paths["patterns"] / out_path
        out_path = io.tagged_bin_path(out_path, self.params)
        self._last_output_path = out_path
        io.write_binary_file(pixeldata, self.params, out_path)
        return out_path

    def _resolve_bin_path(self, filename: str | Path | None = None) -> Path:
        if filename is None:
            if self._last_output_path is None:
                raise ValueError("No output path specified or recorded yet")
            filename = self._last_output_path
        in_path = Path(filename)
        if not in_path.is_absolute() and not in_path.exists():
            in_path = self.cfg.paths["patterns"] / in_path
        if not in_path.exists():
            tagged_path = io.tagged_bin_path(in_path, self.params)
            if tagged_path.exists():
                in_path = tagged_path
        return in_path

    def readbin_events(self, filename: str | Path | None = None) -> dict:
        """Read raw FPGA tick events from an event-driven ``.bin``/``.pat`` file."""
        in_path = self._resolve_bin_path(filename)
        facets, lanes, width, lines_events = io.read_binary_events(in_path)
        return {
            "events": lines_events,
            "metadata": {
                "lanes": lanes,
                "lanewidth": width,
                "facetsinlane": facets,
            },
        }

    def readbin(
        self, filename: str | Path | None = None, grayscale: bool | None = None
    ) -> dict:
        """Read back an event-driven ``.bin``/``.pat`` file into unpacked bits or 8-bit intensities."""
        in_path = self._resolve_bin_path(filename)
        use_gray = self.grayscale if grayscale is None else bool(grayscale)

        if use_gray:
            facets, lanes, width, data = io.read_grayscale_binary_file(
                in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
            )
        else:
            facets, lanes, width, data_bytes = io.read_binary_file(
                in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
            )
            data = np.unpackbits(data_bytes, bitorder=self.bitorder)
        return {
            "data": data,
            "metadata": {
                "lanes": lanes,
                "lanewidth": width,
                "facetsinlane": facets,
            },
        }

    def plot(
        self,
        ptrn_data: dict | None = None,
        filename: str = "plot",
        color_lanes: bool = True,
        view: str = "substrate",
    ) -> np.ndarray:
        """Render an event-driven pattern dictionary (or last written file) to a debug PNG."""
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

    plotptrn = plot


def _cli(
    input_file: str = "combined_grid_test.svg",
    output_file: str | None = None,
    mode: str = "auto",
    correction: bool = False,
    exposures: int = 1,
    plot: bool = True,
    align: str = "laserhead",
):
    """CLI entry point to compile an SVG or raster image into an exposure file."""
    from ..log_setup import configure_logging

    configure_logging(logging.DEBUG)
    engine = ExposureEngine(
        correction=correction,
        exposures=exposures,
        mode=mode,
        placement=SubstratePlacement(align=align),
    )
    duration = engine.generate(input_file, output_path=output_file)
    logger.info(
        f"Exposure complete (mode={engine._last_mode}, duration={duration:.2f} min): "
        f"{engine._last_output_path}"
    )
    pattern_data = engine.readbin()
    if plot:
        engine.plot(pattern_data)


def main():
    fire.Fire(_cli)


if __name__ == "__main__":
    main()
