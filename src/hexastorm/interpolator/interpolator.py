import logging
from pathlib import Path
from io import BytesIO
from time import time
from typing import Union

import numpy as np
from cairosvg import svg2png
from PIL import Image
import cv2

from hexastorm.config import PlatformConfig
from . import geometry
from . import io

# Disable PIL Decompression Bomb Warning
Image.MAX_IMAGE_PIXELS = None


class Interpolator:
    """
    Orchestrates the conversion of digital patterns (SVG/PNG) into binary laser control data.

    This class acts as the bridge between the physical model of the scanner and the
    digital image data. It performs 'Inverse Mapping' interpolation:
    1.  Calculates the physical (X, Y) coordinates of the laser for every clock tick
        using the physics model in `geometry.py`.
    2.  Maps these non-linear, curved scan coordinates onto the rectilinear grid
        of the input image.
    3.  Samples the image at these points to determine if the laser should be ON or OFF.
    """

    def __init__(self):
        self.cfg = PlatformConfig(test=False)

        # Initialize math parameters via config (pure python side)
        raw_params = self.cfg.get_optical_params(correction=True)

        # Initialize the Scanner model (JIT-compiled geometry calculations)
        self.geo = geometry.ScannerModel(raw_params)

        # Link self.params to the JIT class params so we can read them easily
        self.params = self.geo.params

        self.current_dir = Path(__file__).parent.resolve()
        self.debug_folder = self.current_dir / "debug"
        self.bitorder = "big"

    def svgtopil(self, svg_filepath: Path) -> Image.Image:
        """
        Converts SVG vector graphics to a raster PIL Image.
        Includes a -90 degree rotation to align the coordinate system with the laser scan direction.
        """
        with open(svg_filepath, "rb") as f:
            svg_data = f.read()
        dpi = 25.4 / self.params["samplegridsize"]
        png_data = svg2png(bytestring=svg_data, dpi=dpi)
        return Image.open(BytesIO(png_data)).rotate(-90, expand=True)

    def pstopil(self, filepath: Path, pixelsize: float = 0.3527777778) -> Image.Image:
        img = Image.open(filepath)
        scale = pixelsize / self.params["samplegridsize"]
        img.load(scale=scale)
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
            raise Exception("Image is empty")

        # Clip image to the bounding box of non-zero pixels
        nonzero_col = np.argwhere(img_array.sum(axis=0)).squeeze()
        nonzero_row = np.argwhere(img_array.sum(axis=1)).squeeze()

        img_array = img_array[
            nonzero_row[0] : nonzero_row[-1], nonzero_col[0] : nonzero_col[-1]
        ]

        # Update geometry settings based on actual image size
        x_size, y_size = [i * self.params["samplegridsize"] for i in img_array.shape]

        if (x_size > self.params["pltfxsize"]) or (y_size > self.params["pltfysize"]):
            raise Exception("Object does not fit on platform")

        logging.info(f"Sample size is {x_size:.2f} mm by {y_size:.2f} mm")
        self.params["samplexsize"] = x_size
        self.params["sampleysize"] = y_size
        return img_array

    def laser_compensation(self, layerarr: np.ndarray, erode: bool) -> np.ndarray:
        """
        Fast laser spot compensation using OpenCV.
        """
        # Calculate diameter (kernel size must be an integer)
        pixel_radius = self.params["laser_radius_mm"] / self.params["samplegridsize"]
        # Kernel size should be (2 * radius + 1) to ensure a center pixel
        kernel_size = int(round(pixel_radius * 2))

        # Create an elliptical kernel to simulate the circular laser spot
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )

        logging.info(
            f"Fast OpenCV bias: {self.params['laser_radius_mm'] * 1000:.1f}um "
            f"({pixel_radius:.2f} px radius, {kernel_size}px kernel)"
        )

        if erode:
            # cv2.erode expects uint8.
            # It handles binary 0/1 fine, but 0/255 is more standard for CV2.
            result = cv2.erode(layerarr, kernel, iterations=1)
        else:
            result = cv2.dilate(layerarr, kernel, iterations=1)

        return result.astype(np.uint8)

    def patternfile(
        self,
        url: Union[str, Path],
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

        # 1. Load Image
        if file_path.suffix == ".svg":
            pil = self.svgtopil(file_path)
            if test:
                self.debug_folder.mkdir(parents=True, exist_ok=True)
                pil.save(self.debug_folder / "debug.png")
        elif file_path.suffix == ".ps":
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
            img.save(self.debug_folder / "nyquistcheck.png")
            layerarr = np.ones_like(layerarr)  # Force all white for test

        logging.info("Retrieved image")
        logging.info(f"Elapsed {time() - ctime:.2f} seconds")

        # 2. Calculate Coordinates
        # ids is (2, N) array: [y_coords, x_coords] stack.
        # These are the physical integer coordinates on the sample grid.
        ids = self.geo.calculate_coordinates()

        logging.info("Created coordinates for interpolation")
        logging.info(f"Elapsed {time() - ctime:.2f} seconds")

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

        logging.info("Completed interpolation")
        logging.info(f"Elapsed {time() - ctime:.2f} seconds")

        if ptrn.min() < 0 or ptrn.max() > 1:
            raise Exception("This is not a bit list")

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

    def writebin(self, pixeldata: np.ndarray, filename: Union[str, Path] = "test.bin"):
        """Wrapper for io.write_binary_file"""
        # Resolve path: default to debug folder if not absolute
        out_path = Path(filename)
        if not out_path.is_absolute():
            out_path = self.debug_folder / out_path

        io.write_binary_file(pixeldata, self.params, out_path)

    def readbin(self, filename: Union[str, Path] = "test.bin") -> dict:
        """Wrapper for io.read_binary_file"""
        in_path = Path(filename)
        if not in_path.is_absolute():
            in_path = self.debug_folder / in_path

        # Unpack the tuple internally
        facets, lanes, width, data = io.read_binary_file(
            in_path, self.cfg.laser_timing, self.params["bitsinscanline"]
        )

        # Return a clean dictionary
        return {
            "facetsinlane": facets,
            "lanes": lanes,
            "lanewidth": width,
            "data": data,
        }

    def plotptrn(
        self,
        ptrn_data: Union[np.ndarray, dict],
        step: int = 1,
        filename: str = "plot",
        color_lanes: bool = True,
    ):
        """
        Visualizes the generated pattern data.

        Args:
            ptrn_data: The binary pattern data.
            step: Plotting step size (higher is faster but lower resolution).
            filename: Output filename.
            color_lanes: If True, alternates lane colors (Light Blue/White).
        """
        raw_bytes = ptrn_data.get("data")
        metadata = ptrn_data

        # 1. Setup Local Parameters (Start fresh, don't touch self.params)
        # We get the default hardware config (speeds, frequencies, etc.)
        raw_params = self.cfg.get_optical_params(correction=True)
        local_params = geometry.ScannerModel.to_numba_dict(raw_params)

        # A. Calculate the 'lanewidth' for this specific hardware config
        # We calculate this fresh to ensure it matches the JIT's logic
        start_x = geometry._jit_fxpos(0.0, local_params, 0, 0.0)
        end_x = geometry._jit_fxpos(
            local_params["bitsinscanline"] - 1, local_params, 0, 0.0
        )
        calc_lanewidth = (start_x - end_x) * local_params["samplegridsize"]

        # B. Reverse-Engineer 'samplexsize'
        # x_size = lanes * width
        local_params["samplexsize"] = float(metadata["lanes"] * calc_lanewidth)

        # C. Reverse-Engineer 'sampleysize'
        # The formula used to create facets was:
        # facets = ceil( rot_freq * FACETS * (y_size / speed) )
        # So we invert it: y_size = (facets * speed) / (rot_freq * FACETS)
        term = local_params["rotationfrequency"] * local_params["FACETS"]
        local_params["sampleysize"] = float(
            metadata["facetsinlane"] * local_params["stagespeed"] / term
        )

        # Explicitly set these too, just in case
        local_params["lanes"] = float(metadata["lanes"])
        local_params["facetsinlane"] = float(metadata["facetsinlane"])

        # 2. Extract Data
        if isinstance(ptrn_data, np.ndarray):
            raw_bytes = ptrn_data
        elif "data" in ptrn_data:
            raw_bytes = ptrn_data["data"]
            if hasattr(raw_bytes, "values"):
                raw_bytes = raw_bytes.values[0]
        else:
            raise ValueError("Could not find data in input structure")

        ptrn_bits = np.unpackbits(raw_bytes, bitorder=self.bitorder)

        # 3. Get Coordinates (Re-calculate identical grid)
        ids = geometry._jit_calculate_grid(local_params)

        # 4. Handle Downsampling
        factor = int(self.params["downsamplefactor"])
        if factor > 1:
            ids = np.repeat(ids, factor, axis=1)

        # 5. Slicing
        min_len = min(ids.shape[1], len(ptrn_bits))
        x_full = ids[0, :min_len]
        y_full = ids[1, :min_len]
        bits_full = ptrn_bits[:min_len]

        # 6. Prepare Plot Data
        x_plot = x_full[::step]
        y_plot = y_full[::step]
        bits_plot = bits_full[::step]

        x_min, x_max = x_plot.min(), x_plot.max()
        y_min, y_max = y_plot.min(), y_plot.max()

        x_plot -= x_min
        y_plot -= y_min

        width = x_max - x_min + 1
        height = y_max - y_min + 1

        logging.info(f"Plotting Image: {width}x{height} pixels")

        # 7. Rendering
        # Initialize 1-channel canvas for "Lane IDs"
        canvas = np.zeros((width, height), dtype=np.uint8)

        # Define IDs for the grayscale map
        VAL_EVEN = 100
        VAL_ODD = 200
        VAL_ALL = 255

        if color_lanes:
            num_lanes = int(local_params.get("lanes", 1))
            samples_per_lane = len(x_plot) // num_lanes

            # Loop only to assign IDs (Very fast compared to dilation)
            for i in range(num_lanes):
                start = i * samples_per_lane
                end = start + samples_per_lane if i < num_lanes - 1 else len(x_plot)

                # Check if this slice has any active bits
                # (Optimization: boolean indexing is faster than creating sub-arrays)
                lbits = bits_plot[start:end]
                active_indices = np.where(lbits == 1)[0]

                if active_indices.size > 0:
                    # Map local indices back to global indices
                    global_indices = start + active_indices

                    # Determine value (Even=100, Odd=200)
                    val = VAL_EVEN if (i % 2 == 0) else VAL_ODD

                    # Assign to canvas
                    canvas[x_plot[global_indices], y_plot[global_indices]] = val
        else:
            # Standard Monochrome (assign 255 to all active pixels)
            active_mask = bits_plot == 1
            canvas[x_plot[active_mask], y_plot[active_mask]] = VAL_ALL

        # 8. Apply Laser Compensation ONCE
        # This expands the '100' regions and '200' regions simultaneously
        canvas_dilated = self.laser_compensation(canvas, erode=False)

        # 9. Create Final RGB Image
        # Initialize Black Canvas
        rgb_canvas = np.zeros((width, height, 3), dtype=np.uint8)

        if color_lanes:
            # Colors (BGR)
            COL_EVEN = (255, 200, 100)  # Light Blue
            COL_ODD = (255, 255, 255)  # White

            # Map the dilated IDs to Colors
            # Note: Overlaps will favor VAL_ODD (200) because 200 > 100 and dilation uses Max
            rgb_canvas[canvas_dilated == VAL_EVEN] = COL_EVEN
            rgb_canvas[canvas_dilated == VAL_ODD] = COL_ODD
        else:
            # Map all active pixels to White
            rgb_canvas[canvas_dilated > 0] = (255, 255, 255)

        # 10. Rotate and Save
        rotated_canvas = cv2.rotate(rgb_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)

        self.debug_folder.mkdir(parents=True, exist_ok=True)
        save_path = self.debug_folder / f"{filename}.png"
        cv2.imwrite(str(save_path), rotated_canvas)
        logging.info(f"Plot saved to {save_path}")
        return rotated_canvas


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # PCB / photopaper stepsperline single channel, current 130, 2x per line
    fname = "combined_grid_test"
    ctime = time()
    interpolator = Interpolator()
    logging.info(f"Interpolator {time() - ctime:.2f} seconds")
    dir_path = Path(__file__).parent.resolve()
    # postscript resolution test

    url = dir_path / "patterns" / f"{fname}.svg"
    ptrn = interpolator.patternfile(url)
    logging.info(f"Pattern {time() - ctime:.2f} seconds")
    # hexastorm.png pixelsize 0.035
    # url = dir_path / "test-patterns" / "hexastorm.png"
    # ptrn = interpolator.patternfile(url, pixelsize=0.035)
    interpolator.writebin(ptrn, f"{fname}.bin")

    pattern_data = interpolator.readbin(f"{fname}.bin")

    assert np.allclose(
        interpolator.params["lanewidth"], pattern_data["lanewidth"], 1e-3
    )
    assert np.allclose(
        interpolator.params["facetsinlane"], pattern_data["facetsinlane"], 1e-3
    )
    assert len(pattern_data["data"]) == round(
        pattern_data["facetsinlane"]
        * pattern_data["lanes"]
        * np.ceil(interpolator.params["bitsinscanline"] // 8)
    )
    # TODO: step must be an integer!!
    interpolator.plotptrn(pattern_data, step=1)
