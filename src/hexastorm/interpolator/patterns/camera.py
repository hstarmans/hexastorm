import logging
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

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
        pattern_height = 2  # mm

        fig, ax = plt.subplots()

        pitch = self.line_thickness_mm * 2
        num_lines = int(self.view_width / pitch)
        digital_width = self._get_compensated_width(self.line_thickness_mm)

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

    def generate_blank_test(self):
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

    def generate_dot_grid_test(self):
        """
        Generates a 2D grid of dots over the view width and returns the filename.
        Dynamically calculates the number of dots to maintain a minimum 4x spacing.
        """
        filename = f"cam_test_grid_{self.line_thickness_mm * 1000:.0f}um.svg"
        logger.info(f"Generating Dot Grid Test ({filename})...")

        fig, ax = plt.subplots()

        dot_size = self.line_thickness_mm
        digital_dot_diameter = self._get_compensated_width(dot_size)
        dot_radius = digital_dot_diameter / 2.0

        # Calculate how many dots fit horizontally with the minimum spacing criteria
        min_spacing_x = dot_radius * 4.0
        calculated_dots_x = int(self.view_width / min_spacing_x)

        # Enforce maximum 10 dots, or stick with the calculated amount if lower
        if calculated_dots_x > 10:
            num_dots_x = 10
        elif calculated_dots_x < 1:
            raise Exception("View width is too small to comfortably fit even 1 dot!")
        else:
            num_dots_x = calculated_dots_x

        # Recalculate optimal horizontal spacing to evenly fill the view width
        spacing_x = self.view_width / num_dots_x

        # Vertical settings remain fixed per requirements
        num_dots_y = 10
        spacing_y = dot_radius * 4.0
        pattern_height = spacing_y * num_dots_y  # mm

        for i in range(num_dots_x):
            for j in range(num_dots_y):
                if digital_dot_diameter > 0:
                    # Center the dot within its respective grid cell
                    center_x = self.x_min + (i * spacing_x) + (spacing_x / 2.0)
                    center_y = (j * spacing_y) + (spacing_y / 2.0)

                    # Replace Rectangle with a true Circle
                    circle = Circle(
                        (center_x, center_y),
                        radius=dot_radius,
                        color="black",
                        linewidth=0,
                    )
                    ax.add_patch(circle)

        # Use the dynamically calculated pattern_height for this specific canvas
        self._setup_exact_canvas(fig, ax, self.x_max, pattern_height)
        self.resize_to_mm_scale(fig, ax)
        self.save_final(fig, filename)

    def test_view_bounds(self):
        """Generates a test pattern to verify view bounds."""
        scanline_length = self.interpolator.cfg.laser_timing["scanline_length"]
        pattern_bits = self.interpolator.readbin()["data"]
        ptrn = pattern_bits.reshape(-1, scanline_length)
        # --- CLI ASCII Profile ---

        scanline = ptrn[0, :]
        scanline = scanline[::-1]
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
    cam_pat.generate_dot_grid_test()


if __name__ == "__main__":
    main()
