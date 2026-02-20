import logging
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math

from ...config import displacement_kernel
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
        start_point = displacement_kernel(
            pixel=start_pixel, params=self.optical_settings
        )
        self.x_min = -(
            displacement_kernel(
                pixel=start_pixel + px_low, params=self.optical_settings
            )
            - start_point
        )
        self.x_max = -(
            displacement_kernel(
                pixel=start_pixel + px_high, params=self.optical_settings
            )
            - start_point
        )

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

    def generate_vertical_jitter_test(self) -> str:
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

    def generate_horizontal_cross_test(self) -> str:
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


if __name__ == "__main__":
    from ...log_setup import configure_logging

    configure_logging(logging.DEBUG)

    generator = CameraCalibrationGen(
        lower_pixel=200, upper_pixel=500, line_thickness_mm=0.15
    )

    generator.generate_vertical_jitter_test()
    generator.generate_horizontal_cross_test()
