import logging
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

# Disable PIL Decompression Bomb Warning
Image.MAX_IMAGE_PIXELS = None

logger = logging.getLogger(__name__)


class RasterExposure:
    """
    Reference / Raster Pipeline for converting digital patterns (SVG/PNG/JPG/PS) into laser control data.

    Note:
        This class implements the frozen bitmap-based inverse-mapping pipeline
        (rasterizing SVGs via `resvg-py` or loading bitmaps via PIL onto a 2D pixel
        grid and sampling via `geometry._jit_calculate_grid`). It is maintained as the
        **golden reference implementation** for verifying the direct vector-to-timestamp
        pipeline (`vector.VectorExposure`) and as the fallback for raster images.
        New vector/KiCad features and laserhead coordinate transforms belong in `vector.py`.
    """

    def __init__(
        self,
        correction: bool | dict = False,
        exposures: int = 1,
        mount: LaserheadMount | None = None,
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
        self._last_coord: CoordinateTransform | None = None
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

    def pstopil(self, filepath: Path, pixelsize: float = 0.3527777778) -> Image.Image:
        img = Image.open(filepath)
        scale = pixelsize / self.params["samplegridsize"]
        if isinstance(img, EpsImagePlugin.EpsImageFile):
            img.load(scale=max(1, round(scale)))
        else:
            img.load()
        return img

    def imgtopil(self, filepath: Path, pixelsize: float) -> Image.Image:
        img = Image.open(filepath)
        scale = pixelsize / self.params["samplegridsize"]
        img = img.resize([round(x * scale) for x in img.size])
        return img

    def piltoarray(self, img: Image.Image) -> np.ndarray:
        """Convert PIL Image (unrotated Substrate frame) to boolean/binary numpy array."""
        img_array = np.array(img.convert("1", dither=Image.Dither.NONE))
        if img_array.max() == 0:
            raise ValueError("Image is empty")

        grid_size = float(self.params["samplegridsize"])
        h_px, w_px = img_array.shape
        width_mm = w_px * grid_size
        height_mm = h_px * grid_size

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

    def laser_compensation(self, layerarr: np.ndarray, erode: bool) -> np.ndarray:
        """
        Elliptical laser spot morphology compensation.
        """
        return plotter.laser_compensation(layerarr, self.params, erode=erode)

    def patternfile(
        self,
        url: str | Path,
        pixelsize: float = 0.3527777778,
        test: bool = False,
        positiveresist: bool = False,
        laser_compensate: bool = False,  # remove laser spot size
        radius_mm: float = 0.015,  # 15um radius = 30um diameter
    ) -> np.ndarray:
        """
        Generates the binary laser pattern by sampling the image at calculated laser positions.

        The process handles the non-linear distortion of the prism scanner by:
        1. Generating a lookup table of coordinates (ids) where the laser will physically be.
        2. Using these coordinates to index into the image array.

        Args:
            url: Path to the input image pattern.
            pixelsize: Size of pixels in mm if using non-SVG input.
            test: If True, saves debug images for verification.
        """
        ctime = time()
        file_path = Path(url)
        if not file_path.is_absolute() and not file_path.exists():
            svg_candidate = self.cfg.paths["svgs"] / file_path
            img_candidate = self.cfg.paths["images"] / file_path
            if svg_candidate.exists():
                file_path = svg_candidate
            elif img_candidate.exists():
                file_path = img_candidate
            else:
                file_path = svg_candidate

        # 1. Load Image
        suffix = file_path.suffix.lower()
        if suffix == ".svg":
            pil = self.svgtopil(file_path)
            if test:
                self.debug_folder.mkdir(parents=True, exist_ok=True)
                pil.save(self.debug_folder / "read_debug.png")
        elif suffix == ".ps":
            pil = self.pstopil(file_path)
        else:
            pil = self.imgtopil(file_path, pixelsize)

        layerarr = self.piltoarray(pil).astype(np.uint8)
        # laser is black in the current images
        if laser_compensate:
            layerarr = self.laser_compensation(layerarr, erode=False)
            Image.fromarray(layerarr.astype(np.uint8) * 255).save(
                self.debug_folder / "erosioncheck.png"
            )
        if test:
            self.debug_folder.mkdir(parents=True, exist_ok=True)
            img = Image.fromarray(layerarr.astype(np.uint8) * 255)
            img.save(self.debug_folder / "simplecheck.png")

        logger.debug("Retrieved image")
        logger.debug(f"Elapsed {time() - ctime:.2f} seconds")

        # 2 & 3. Map Laserhead Coordinates -> Substrate Bitmap via CoordinateTransform
        coord = self._last_coord
        h_px, w_px = layerarr.shape
        grid_size = float(self.params["samplegridsize"])

        if coord is None or coord.is_identity_lh_to_sub:
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

                x_sub_mm, y_sub_mm = coord.laserhead_to_substrate_xy(u_mm, v_mm)
                in_bounds = (
                    (x_sub_mm >= 0.0)
                    & (x_sub_mm <= coord.width_mm)
                    & (y_sub_mm >= 0.0)
                    & (y_sub_mm <= coord.height_mm)
                )
                col = np.clip(
                    np.floor(x_sub_mm[in_bounds] / grid_size).astype(np.int32),
                    0,
                    w_px - 1,
                )
                row = np.clip(
                    np.floor(
                        (coord.height_mm - y_sub_mm[in_bounds]) / grid_size
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

        if ptrn.min() < 0 or ptrn.max() > 1:
            raise ValueError("This is not a bit list")

        # 4. Handle Polarity and Padding
        # 'mask' defines the valid image area. '~mask' is the padding/overscan area.
        if not self.params["positiveresist"]:
            # Negative Resist: Invert image.
            # Force padding to 0 (Laser OFF) to avoid "white edge" artifacts.
            ptrn = np.logical_not(ptrn)
            ptrn[~mask] = 0
        else:
            # Positive Resist: Keep image polarity.
            # WARNING: This logic sets padding to 1. Ensure this is intended behavior
            # (i.e., you want the laser ON outside the image area).
            ptrn[~mask] = 1

        ptrn = np.repeat(ptrn, self.params["downsamplefactor"])
        ptrn = np.packbits(ptrn, bitorder=self.bitorder)
        return ptrn

    def img_to_bin(
        self,
        img_name: str | Path,
        bin_name: str | Path | None = None,
        camera: bool = False,
        **kwargs,
    ) -> float:
        """Convenience method to convert IMG directly to BIN and return job duration in minutes."""
        img_name = Path(img_name)

        if not bin_name:
            bin_name = img_name.with_suffix(".pat")

        # Generate the packed bits
        ptrn = self.patternfile(img_name, **kwargs)

        # Write to disk
        self.writebin(ptrn, bin_name)

        # Calculate and log duration statelessly
        duration_min = self.estimate_job_duration(ptrn, camera=camera)
        device = "camera" if camera else "machine"
        logger.info(
            f"Estimated Job Duration: {duration_min:.2f} minutes using {device}"
        )

        return duration_min

    def estimate_job_duration(
        self, ptrn_packed: np.ndarray, camera: bool = False
    ) -> float:
        """
        Calculates expected job duration in minutes directly from the pattern array.
        """
        if camera:
            # Unpack the bytes back to bits to easily check line by line
            data_bits = np.unpackbits(ptrn_packed, bitorder=self.bitorder)
            bits_in_scanline = int(self.params["bitsinscanline"])

            # Reshape into a 2D array: (total_lines, bits_per_line)
            lines2d = data_bits.reshape(-1, bits_in_scanline)

            # Count rows where the sum of bits is greater than 0
            active_lines = np.count_nonzero(lines2d.sum(axis=1) > 0)

            # 3 lines per second = 180 lines per minute
            total_min = active_lines / 180.0

        else:
            # Machine physical calculation
            laz_tim = self.cfg.laser_timing
            lanes = self.params["lanes"]
            facets = self.params["facetsinlane"]

            line_min = laz_tim["facets"] * laz_tim["rpm"]

            expose_time = (lanes * facets) / line_min
            startup_time = 0.8
            lane_change_time = lanes * 0.2

            total_min = startup_time + expose_time + lane_change_time

        return total_min

    def _tagged_bin_path(self, path: Path) -> Path:
        """Appends exposure count and optical-correction tag to the output stem."""
        return io.tagged_bin_path(path, self.params)

    def writebin(self, pixeldata: np.ndarray, filename: str | Path = "test.bin"):
        """Wrapper for io.write_binary_file.

        Encodes exposure count and optical-correction flag into the output filename
        stem so the firmware can validate the file contract without parsing binary:
          e.g. ``myjob_e1_nocor.pat`` or ``myjob_e4_cor.pat``
        """
        out_path = Path(filename)
        if not out_path.is_absolute():
            out_path = self.cfg.paths["patterns"] / out_path

        out_path = self._tagged_bin_path(out_path)

        self._last_output_path = out_path
        io.write_binary_file(
            pixeldata,
            self.params,
            out_path,
            invert_hw_scan_dir=self.mount.invert_hw_scan_dir,
        )

    def readbin(self, filename: str | Path | None = None) -> dict:
        """Wrapper for io.read_binary_file"""

        if filename is None:
            filename = self._last_output_path
        in_path = Path(filename)
        if not in_path.is_absolute():
            in_path = self.cfg.paths["patterns"] / in_path
        if not in_path.exists():
            tagged_path = self._tagged_bin_path(in_path)
            if tagged_path.exists():
                in_path = tagged_path

        # Unpack the tuple internally
        facets, lanes, width, data_bytes = io.read_binary_file(
            in_path, self.cfg.laser_timing, int(self.params["bitsinscanline"])
        )

        data_bits = np.unpackbits(data_bytes, bitorder=self.bitorder)

        # Return a clean dictionary
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
