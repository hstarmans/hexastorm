import logging
import os

import matplotlib

# Use Agg for file generation without a window popping up
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from ...config import PlatformConfig
from ..interpolator import Interpolator


logger = logging.getLogger(__name__)


class LaserCalibrationGen:
    """
    Generates SVG calibration patterns for laser scanning systems.

    This class creates precise vector graphics for testing galvanometer
    or polygon mirror scanners, including:
    1. Fan Test (Radial resolution and focus)
    2. Wedge Jitter Test (Traditional vertical jitter test)
    3. Combined Grid Test (Simultaneous X-axis jitter and Y-axis cross-scan)
    """

    def __init__(
        self,
        pix_per_mm: int = 200,
        correction: bool = False,
        exposures: int = 1,
    ):
        """
        Initialize the generator.

        Args:
            pix_per_mm (int): Resolution of the internal rasterization for calculation
                              (does not affect vector output precision). Defaults to 200.
        """
        config = PlatformConfig(test=False)

        self.interpolator = Interpolator(correction=correction, exposures=exposures)
        # Determine output directory
        self.script_directory = config.paths["svgs"]

        # Ensure directory exists
        os.makedirs(self.script_directory, exist_ok=True)

        self.pix_per_mm = pix_per_mm

        self.optical_settings = config.get_optical_params()

        # The code will automatically shrink vector features
        # by this amount to prevent overlap (kerf compensation).
        self.laser_spot_mm = self.optical_settings["laser_radius_mm"] * 2

        self.setup_style()
        self.tight = "tight"  # use None to disable

        logger.info(f"Initialized. Results will be saved to: {self.script_directory}")
        logger.debug(f"Resolution set to: {self.pix_per_mm} pixels/mm")
        logger.debug(
            f"Laser Spot Compensation: {self.laser_spot_mm * 1000:.1f} microns ({self.laser_spot_mm} mm)"
        )

    def set_correction(self, correction: bool = None, exposures: int = None):
        """Enables or disables correction mode."""
        self.interpolator.set_optical_params(correction=correction, exposures=exposures)

    def setup_style(self):
        """Sets up the matplotlib style for technical drawings."""
        # RECOMMENDED: 2.0mm is a sweet spot for 30x30mm calibration grids
        text_size = 2.0  # mm (font height)
        tick_size = 0.3  # mm (slightly thicker for visibility)

        # Convert mm to typographic points (1 inch = 25.4 mm = 72 points)
        mm_to_points = 72 / 25.4
        self.points_text = text_size * mm_to_points
        self.points_ticks = tick_size * mm_to_points

        plt.rcParams.update(
            {
                "font.family": "sans-serif",
                "font.size": self.points_text,
                "axes.labelsize": self.points_text * 1.1,  # Slightly larger labels
                "xtick.labelsize": self.points_text,
                "ytick.labelsize": self.points_text,
                "xtick.major.size": self.points_ticks * 4,  # Length of the tick line
                "ytick.major.size": self.points_ticks * 4,
                "xtick.major.width": 0.2,  # Thickness of the tick line
                "ytick.major.width": 0.2,
                "grid.linewidth": 0.1,
                "lines.linewidth": 0.5,
            }
        )

    def _get_compensated_width(self, target_width_mm):
        """
        Calculates the digital width required to achieve the physical target width.

        Args:
            target_width_mm (float): The desired physical line width.

        Returns:
            float: The width to draw in the SVG. If the target is smaller
                   than the laser spot, returns 0.0.
        """
        digital_width = target_width_mm - self.laser_spot_mm
        if digital_width <= 0:
            return 0.0
        return digital_width

    def resize_to_mm_scale(self, fig, ax):
        """
        Resizes figure so that 1 Data Unit = 1 mm.
        Must be called BEFORE calculating screen-relative positions.
        """
        # 1. Setup temporary DPI for calculation
        calc_dpi = 100
        fig.set_dpi(calc_dpi)
        fig.canvas.draw()

        # 2. Get current pixels per unit
        p0 = ax.transData.transform((0, 0))
        p1 = ax.transData.transform((1, 1))
        current_x_pix_per_unit = abs(p1[0] - p0[0])

        # 3. Calculate Scale (1 unit = 1 mm = 1/25.4 inches)
        scale_x = (1 / 25.4) / (current_x_pix_per_unit / calc_dpi)

        # 4. Apply Scaling
        curr_w, curr_h = fig.get_size_inches()
        fig.set_size_inches(curr_w * scale_x, curr_h * scale_x)

        # 5. Set Final DPI
        final_dpi = round(self.pix_per_mm * 25.4)
        fig.set_dpi(final_dpi)

        # 6. Final Draw to update transforms
        fig.canvas.draw()
        return final_dpi

    def _add_line(self, ax, x1, y1, x2, y2, width=0.1):
        line = Line2D([x1, x2], [y1, y2], linewidth=width, color="black", clip_on=False)
        ax.add_line(line)

    def add_stitched_vernier(self, ax, center_x, center_y, vertical=False):
        """
        Draws a Vernier pattern specifically designed for Slicing overlaps.

        It clips the pattern:
        - The Master (Lower) scale exists ONLY to the Left of center_x.
        - The Slave (Upper) scale exists ONLY to the Right of center_x.
        """

        # Settings
        num_ticks = 10  # Enough ticks to see the pattern fade in/out
        pitch = 1.0  # Spacing between lines
        tick_height = 1.0
        width = 0.2  # line-width

        # We iterate a range that spans across the boundary
        # e.g. from -7 to +7
        offset_range = range(-(num_ticks // 2), (num_ticks // 2) + 1)

        for i in offset_range:
            # 1. Calculate Potential Positions assuming 0 is at center_x
            #    (We calculate BOTH potential positions for every index to see what fits)

            # Master (Standard) position
            pos_master = center_x + (i * pitch)

            # Slave (Vernier) position
            # Note: We align them at index 0 (the boundary)
            pos_slave = center_x + (i * pitch)

            # 2. Logic for Horizontal Orientation (Vertical=False)
            if not vertical:
                # --- MASTER SCALE (Lower) ---
                # Rule: Only draw IF it sits to the LEFT of the slicer cut (center_x)
                if pos_master <= center_x:
                    self._add_line(
                        ax,
                        pos_master,
                        center_y,
                        pos_master,
                        center_y - tick_height,
                        width=width,
                    )

                # --- SLAVE SCALE (Upper) ---
                # Rule: Only draw IF it sits to the RIGHT of the slicer cut (center_x)
                if pos_slave >= center_x:
                    self._add_line(
                        ax,
                        pos_slave,
                        center_y,
                        pos_slave,
                        center_y + tick_height,
                        width=width,
                    )

            # 3. Logic for Vertical Orientation (Vertical=True)
            else:
                # Same logic but coordinates flipped
                if pos_master <= center_y:  # Below split
                    self._add_line(
                        ax,
                        center_x,
                        pos_master,
                        center_x - tick_height,
                        pos_master,
                        width=width,
                    )
                if pos_slave >= center_y:  # Above split
                    self._add_line(
                        ax,
                        center_x,
                        pos_slave,
                        center_x + tick_height,
                        pos_slave,
                        width=width,
                    )

    def add_vernier_alignment(self, ax, start_pos, boundary_level, vertical=False):
        """Standard alignment vernier."""
        # Master Scale
        for i in range(11):
            pos = start_pos + i
            if not vertical:
                self._add_line(
                    ax, pos, boundary_level - 1.0, pos, boundary_level, width=0.15
                )
            else:
                self._add_line(
                    ax, boundary_level - 1.0, pos, boundary_level, pos, width=0.15
                )

        # Slave Scale
        for i in range(11):
            pos = start_pos + (i * 0.95)
            if not vertical:
                self._add_line(
                    ax, pos, boundary_level, pos, boundary_level + 1.0, width=0.15
                )
            else:
                self._add_line(
                    ax, boundary_level, pos, boundary_level + 1.0, pos, width=0.15
                )

    def get_top_left_in_data_coords(self, fig, ax):
        """
        Returns the (X, Y) position in mm of the axis origin assuming top left is (0,0).
        """
        renderer = fig.canvas.get_renderer()

        # This is the box (in inches) that will actually be saved
        tight_bbox = fig.get_tightbbox(renderer)

        # Get the Data Origin (0,0) in Figure Pixels
        origin_pix = ax.transData.transform((0, 0))

        # Convert Data Origin to Figure Inches
        origin_inch_x = origin_pix[0] / fig.dpi
        origin_inch_y = origin_pix[1] / fig.dpi

        final_origin_x_mm = origin_inch_x * 25.4
        # Matplotlib (Bottom-Up) Y Origin
        final_origin_y_mm_bottom = origin_inch_y * 25.4

        if self.tight == "tight":
            final_origin_x_mm -= tight_bbox.x0 * 25.4
            final_origin_y_mm_bottom -= tight_bbox.y0 * 25.4
            # Calculate full image size
            full_w_mm = tight_bbox.width * 25.4
            full_h_mm = tight_bbox.height * 25.4
        else:
            full_w_mm = fig.get_size_inches()[0] * 25.4
            full_h_mm = fig.get_size_inches()[1] * 25.4

        # Calculate Top-Down Y Origin (for Inkscape/Machine Config)
        final_origin_y_mm_top = full_h_mm - final_origin_y_mm_bottom

        logger.debug(f"IMAGE SIZE: {full_w_mm:.3f}mm x {full_h_mm:.3f}mm")
        logger.debug(
            f"ORIGIN (Screen/Top-Left):       X={final_origin_x_mm:.4f}, Y={final_origin_y_mm_top:.4f}"
        )
        return final_origin_x_mm, final_origin_y_mm_top

    def save_final(self, fig, filename):
        """Just saves the file, assuming resizing is already done."""
        output_path = os.path.join(self.script_directory, filename)
        # padding=0 ensures the edges match what we calculated
        fig.savefig(output_path, dpi=fig.dpi, pad_inches=0, bbox_inches=self.tight)
        plt.close(fig)
        logger.info(f"Saved {filename}")
        self.interpolator.img_to_bin(filename)

    def _clean_axes(self, ax, x_label: str, y_label: str):
        """
        Removes spines (borders) and adds standardized axis labels.

        Args:
            ax (matplotlib.axes.Axes): The axes to clean.
            x_label (str): Label for the X-axis.
            y_label (str): Label for the Y-axis.
        """
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        for key, spine in ax.spines.items():
            spine.set_visible(False)

    def generate_combined_test(self):
        """
        Generates a Combined Grid Test for Jitter and Cross-Scan errors.

        This creates a single square grid (default 30x30mm) containing:
        1. Vertical Lines (Columns): Varying thickness to test X-axis (fast) jitter.
        2. Horizontal Lines (Rows): Varying thickness to test Y-axis (slow) cross-scan error.

        Labels are placed on the outside of the grid, and a minimal layout
        is used (no grid lines, only boundary ticks) to prevent visual clutter.
        """
        logger.debug("Generating Combined Grid Test (Compensated)...")
        box_size = 25  # mm (Square size)
        cell_size = 5  # mm (Size of each thickness zone)
        linewidth_start = 0.06  # mm
        stepsize = 0.020  # mm
        lines_per_group = 10  # Number of lines per grid cell

        fig, ax = plt.subplots()

        def get_thickness(n):
            return n * stepsize + linewidth_start

        num_steps = int(box_size // cell_size)
        tick_locs = []
        tick_labels = []

        lanewidth = self.optical_settings["lanewidth"]

        for i in range(num_steps):
            target_thickness = get_thickness(i)
            digital_thickness = self._get_compensated_width(target_thickness)

            pos_start = i * cell_size
            tick_locs.append(pos_start + cell_size / 2)
            tick_labels.append(f"{target_thickness * 1000:.0f}")

            # CALCULATE LAYOUT BASED ON PHYSICAL (TARGET) DIMENSIONS
            # The group width must include the empty space (gaps)
            # We assume Pitch = 2 * thickness (Line + Gap)
            pitch = target_thickness * 2

            # Total width of the group of lines physically
            # = (lines-1)*pitch + 1*line_width
            group_physical_span = (lines_per_group - 1) * pitch + target_thickness

            # Center the physical group in the cell
            center_offset_x = (cell_size - group_physical_span) / 2
            start_x = pos_start + center_offset_x

            # --- Vertical Lines (Fast Axis) ---
            for line_idx in range(lines_per_group):
                if digital_thickness > 0:
                    # Calculate CENTER of the line based on PHYSICAL pitch
                    line_center_x = (
                        start_x + (pitch * line_idx) + (target_thickness / 2)
                    )

                    # Draw Rectangle based on CENTER and DIGITAL width
                    rect_left = line_center_x - (digital_thickness / 2)

                    rect = Rectangle(
                        (rect_left, 0),
                        digital_thickness,
                        box_size,
                        linewidth=0,
                        edgecolor="none",
                        facecolor="black",
                        alpha=1,
                    )
                    ax.add_patch(rect)

            # --- Horizontal Lines (Slow Axis) ---
            # Use same centering logic for Y
            center_offset_y = (cell_size - group_physical_span) / 2
            start_y = pos_start + center_offset_y

            for line_idx in range(lines_per_group):
                if digital_thickness > 0:
                    line_center_y = (
                        start_y + (pitch * line_idx) + (target_thickness / 2)
                    )
                    rect_bottom = line_center_y - (digital_thickness / 2)

                    rect = Rectangle(
                        (0, rect_bottom),
                        box_size,
                        digital_thickness,
                        linewidth=0,
                        edgecolor="none",
                        facecolor="black",
                        alpha=1,
                    )
                    ax.add_patch(rect)

        ax.set_xticks(tick_locs)
        ax.set_xticklabels(tick_labels, rotation=-90)
        ax.set_yticks(tick_locs)
        ax.set_yticklabels(tick_labels)

        ax.set_xlabel("scan --> width [μm]", x=0.3)
        ax.set_ylabel("stage --> width [μm]", y=0.2)

        # Needed, so the grid is not distorted
        ax.set_aspect("equal")

        ax.set_xlim(0, box_size)
        ax.set_ylim(0, box_size)

        for key, spine in ax.spines.items():
            spine.set_visible(False)

        ax.grid(False)
        ax.tick_params(axis="both", which="major", length=5, width=1)
        ax.tick_params(axis="both", which="minor", length=0)

        # 1. First, Resize the figure to force 1mm scale
        self.resize_to_mm_scale(fig, ax)

        # 2. Calculate where the Top-Left of the paper is in your Data Coordinates
        tl_x, tl_y = self.get_top_left_in_data_coords(fig, ax)

        # Vernier Aligntment Mark for Stage
        self.add_vernier_alignment(
            ax, start_pos=-10, boundary_level=lanewidth - tl_x, vertical=True
        )

        # Vernier Alignment Mark for Scan
        self.add_stitched_vernier(
            ax, center_x=lanewidth - tl_x, center_y=20, vertical=False
        )

        # 4. Save
        self.save_final(fig, "combined_grid_test.svg")


if __name__ == "__main__":
    from ...log_setup import configure_logging

    configure_logging(logging.DEBUG)
    generator = LaserCalibrationGen()

    # Generate all patterns
    generator.generate_combined_test()
    logger.debug("All patterns generated successfully.")
