import logging
from io import BytesIO
from pathlib import Path
from time import time

import numpy as np
from PIL import EpsImagePlugin, Image

from hexastorm.config import PlatformConfig

from . import geometry, io, plotter

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

    def __init__(self, correction: bool = False, exposures: int = 1):
        self.cfg = PlatformConfig(test=False)
        self.correction = correction
        self.exposures = exposures
        self.set_optical_params(correction=correction, exposures=exposures)
        self.debug_folder = self.cfg.paths["base"] / "debug"
        self.debug_folder.mkdir(parents=True, exist_ok=True)
        self._last_output_path = None  # To track the last output pat file for reading
        self.bitorder = "big"

    def set_optical_params(
        self, correction: bool | None = None, exposures: int | None = None
    ):
        if correction is not None:
            self.correction = correction
        if exposures is not None:
            self.exposures = exposures
        # Initialize math parameters via config (pure python side)
        raw_params = self.cfg.get_optical_params(
            correction=self.correction, exposures=self.exposures
        )
        # Initialize the Scanner model (JIT-compiled geometry calculations)
        self.geo = geometry.ScannerModel(raw_params)

        # Link self.params to the JIT class params so we can read them easily
        self.params = self.geo.params

    def svgtopil(self, svg_filepath: Path) -> Image.Image:
        """
        Converts SVG vector graphics to a raster PIL Image using resvg-py.
        Includes a -90 degree rotation to align the coordinate system with the laser scan direction.
        """
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
        return img.rotate(-90, expand=True)

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
        """
        Converts PIL Image to a numpy array and crops it to the region of interest.
        Also updates the system parameters (samplexsize/sampleysize) to match the actual image dimensions.
        """
        img_array = np.array(img.convert("1"))
        if img_array.max() == 0:
            raise ValueError("Image is empty")

        # Update geometry settings based on actual image size
        x_size, y_size = [i * self.params["samplegridsize"] for i in img_array.shape]

        if (x_size > self.params["pltfxsize"]) or (y_size > self.params["pltfysize"]):
            raise ValueError("Object does not fit on platform")

        logger.debug(f"Sample size is {x_size:.2f} mm by {y_size:.2f} mm")
        self.params["samplexsize"] = x_size
        self.params["sampleysize"] = y_size
        return img_array

    def laser_compensation(self, layerarr: np.ndarray, erode: bool) -> np.ndarray:
        """
        Fast laser spot compensation using OpenCV.
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

        # 2. Calculate Coordinates
        # ids is (2, N) array: [y_coords, x_coords] stack.
        # These are the physical integer coordinates on the sample grid.
        ids = self.geo.calculate_coordinates()

        logger.debug("Created coordinates for interpolation")
        logger.debug(f"Elapsed {time() - ctime:.2f} seconds")

        # Note: Coordinate convention swap.
        # The 'geometry' module returns stacked arrays, but the interpretation here
        # assumes ids[0] is the Y-index (Row) and ids[1] is the X-index (Column).
        y_coords = ids[0]
        x_coords = ids[1]

        # 3. Map Image to Coordinates (Sampling)
        # Create a boolean mask for coordinates that fall inside the image bounds.
        # The scanner area is typically larger than the image (overscan).
        mask = (
            (x_coords >= 0)
            & (x_coords < layerarr.shape[1])
            & (y_coords >= 0)
            & (y_coords < layerarr.shape[0])
        )

        ptrn = np.zeros(x_coords.shape, dtype=np.uint8)

        # Advanced Indexing: Sample the image at the calculated coordinates.
        # layerarr[row, col] corresponds to layerarr[y, x].
        ptrn[mask] = layerarr[y_coords[mask], x_coords[mask]]

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
            scanline_length = self.cfg.laser_timing["scanline_length"]

            # Reshape into a 2D array: (total_lines, bits_per_line)
            lines2d = data_bits.reshape(-1, scanline_length)

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
        io.write_binary_file(pixeldata, self.params, out_path)

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
    ) -> np.ndarray:
        """Visualizes the generated pattern data via :mod:`hexastorm.exposure.plotter`."""
        return plotter.plot_pattern(
            ptrn_data,
            cfg=self.cfg,
            correction=self.correction,
            exposures=self.exposures,
            filename=filename,
            color_lanes=color_lanes,
        )
