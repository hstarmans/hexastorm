import logging
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math

from .machine import LaserCalibrationGen

logger = logging.getLogger(__name__)


class CameraCalibrationGen(LaserCalibrationGen):
    """
    Generates calibration patterns specifically for a fixed Camera Viewport.
    Ensures the coordinate system starts at absolute X=0 (machine origin),
    preserving whitespace to the left of the camera view.
    """

    def __init__(
        self,
        lower_pixel: int,
        upper_pixel: int,
        line_thickness_mm: float = 0.15,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.pattern_height_mm = 2.0  # Height of the test pattern in mm

        # IMPORTANT: Disable 'tight' cropping.
        # We will manually control the canvas size to include the whitespace at X=0.
        self.tight = None

        self.line_thickness_mm = line_thickness_mm
        self.calculate_view_bounds(lower_pixel, upper_pixel)

    def calculate_view_bounds(self, px_low: int, px_high: int):
        start_pixel = self.optical_settings["startpixel"]
        grid_size = self.optical_settings["samplegridsize"]
        start_point = self.interpolator.geo.fxpos(pixel=start_pixel)

        def px_to_mm(px):
            return (start_point - self.interpolator.geo.fxpos(pixel=px)) * grid_size

        # 3. Calculated bounds
        self.x_min = px_to_mm(px_low)
        self.x_max = px_to_mm(px_high)
        self.view_width = self.x_max - self.x_min

    def _setup_exact_canvas(self, fig, ax, width_mm, height_mm):
        """
        Configures the figure to match the physical dimensions exactly,
        starting from X=0 and Y=0.
        """
        ax.set_xlim(0, width_mm)
        ax.set_ylim(0, height_mm)
        ax.axis("off")
        ax.margins(0)
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        ax.set_position([0, 0, 1, 1])

    def generate_vertical_jitter_test(self):
        """Generates test and returns the filename used."""
        filename = f"cam_test_vertical_{self.line_thickness_mm * 1000:.0f}um.svg"
        logger.info(f"Generating Vertical Jitter Test ({filename})...")

        fig, ax = plt.subplots()

        pitch = self.line_thickness_mm * 2
        num_lines = int(self.view_width / pitch)
        digital_width = self._get_compensated_width(self.line_thickness_mm)
        pattern_height = self.pattern_height_mm

        for i in range(num_lines):
            start_x = self.x_min + (i * pitch)
            if digital_width > 0:
                center_x = start_x + (self.line_thickness_mm / 2)
                draw_x = center_x - (digital_width / 2)
                rect = Rectangle(
                    (draw_x, 0),
                    digital_width,
                    pattern_height,
                    color="black",
                    linewidth=0,
                )
                ax.add_patch(rect)

        self._setup_exact_canvas(fig, ax, self.x_max, pattern_height)
        self.resize_to_mm_scale(fig, ax)
        self.save_final(fig, filename)

    def generate_blank_test(self) -> str:
        """
        Generates a test pattern to verify view bounds.
        Fills the area from 0 to x_min with black, and leaves the area
        between x_min and x_max completely blank.
        """
        filename = "cam_test_blank_bounds.svg"
        logger.info(f"Generating Blank Bounds Test ({filename})...")

        fig, ax = plt.subplots()

        # 1. Draw a solid block from X=0 to X=x_min
        # This makes the start of the camera viewport highly visible.

        rect_left = Rectangle(
            (self.x_min, 0),
            self.x_max - self.x_min,
            self.pattern_height_mm,
            color="black",
            linewidth=0,
        )
        ax.add_patch(rect_left)

        self._setup_exact_canvas(fig, ax, self.x_max, self.pattern_height_mm)
        self.resize_to_mm_scale(fig, ax)
        self.save_final(fig, filename)

    def generate_horizontal_cross_test(self):
        """Generates test and returns the filename used."""
        filename = f"cam_test_horizontal_{self.line_thickness_mm * 1000:.0f}um.svg"
        logger.info(f"Generating Horizontal Cross Test ({filename})...")

        fig, ax = plt.subplots()

        pitch = self.line_thickness_mm * 2
        digital_width = self._get_compensated_width(self.line_thickness_mm)
        num_lines = math.ceil(self.pattern_height_mm / pitch)

        for i in range(num_lines):
            start_y = i * pitch
            if digital_width > 0:
                center_y = start_y + (self.line_thickness_mm / 2)
                draw_y = center_y - (digital_width / 2)
                rect = Rectangle(
                    (self.x_min, draw_y),
                    self.view_width,
                    digital_width,
                    color="black",
                    linewidth=0,
                )
                ax.add_patch(rect)

        self._setup_exact_canvas(fig, ax, self.x_max, self.pattern_height_mm)
        self.resize_to_mm_scale(fig, ax)
        self.save_final(fig, filename)

    def test_view_bounds(self):
        """Generates a test pattern to verify view bounds."""
        self.generate_blank_test()
        scanline_length = self.interpolator.cfg.laser_timing["scanline_length"]
        pattern_bits = self.interpolator.readbin()["data"]
        ptrn = pattern_bits.reshape(-1, scanline_length)
        # --- CLI ASCII Profile ---

        scanline = ptrn[0, :]
        bucket_size = 100

        logger.info(f"Exact sum in target region (200:500): {scanline[200:500].sum()}")
        logger.info("-" * 50)

        logger.info(f"{'Pixel Range':<15} | {'Sum':<6} | {'Profile'}")
        logger.info("-" * 50)

        # Loop through the scanline in chunks of 100
        for i in range(0, len(scanline), bucket_size):
            end = min(i + bucket_size, len(scanline))
            bucket = scanline[i:end]
            bucket_sum = bucket.sum()

            # Create a simple ASCII bar (scales to max 20 '#' characters)
            # Assuming the values are 0 or 1. If your active pixels are 255,
            # change the math to: bucket_sum / (255 * bucket_size)
            intensity_ratio = bucket_sum / bucket_size
            bar = "#" * int(intensity_ratio * 20)

            # Print the row
            logger.info(f"{i:04d} to {end:04d} | {bucket_sum:<6.0f} | {bar}")


def main():
    from ...log_setup import configure_logging

    configure_logging(logging.DEBUG)

    cam_pat = CameraCalibrationGen(
        lower_pixel=200, upper_pixel=500, line_thickness_mm=0.15
    )
    cam_pat.generate_vertical_jitter_test()
    cam_pat.generate_horizontal_cross_test()


if __name__ == "__main__":
    main()
