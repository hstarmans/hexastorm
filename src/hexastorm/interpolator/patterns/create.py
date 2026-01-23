import logging
import os
import sys
from typing import Optional

import matplotlib
import numpy as np

# Use Agg for file generation without a window popping up
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib.transforms import Affine2D

# Configure standard logger
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
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
        output_dir: Optional[str] = None,
        pix_per_mm: int = 200,
        laser_spot_size_microns: float = 30.0,
    ):
        """
        Initialize the generator.

        Args:
            output_dir (str, optional): Directory to save generated SVGs.
                                      Defaults to the script's directory.
            pix_per_mm (int): Resolution of the internal rasterization for calculation
                              (does not affect vector output precision). Defaults to 200.
            laser_spot_size_microns (float): The physical diameter of the laser spot in microns.
                                             The code will automatically shrink vector features
                                             by this amount to prevent overlap (kerf compensation).
        """
        # Determine output directory
        if output_dir:
            self.script_directory = output_dir
        else:
            self.script_directory = os.path.dirname(os.path.abspath(__file__))

        # Ensure directory exists
        os.makedirs(self.script_directory, exist_ok=True)

        self.pix_per_mm = pix_per_mm

        # Convert microns to mm for internal calc
        self.laser_spot_mm = laser_spot_size_microns / 1000.0

        self.setup_style()

        logger.info(f"Initialized. Results will be saved to: {self.script_directory}")
        logger.info(f"Resolution set to: {self.pix_per_mm} pixels/mm")
        logger.info(
            f"Laser Spot Compensation: {laser_spot_size_microns} microns ({self.laser_spot_mm} mm)"
        )

    def setup_style(self):
        """Sets up the matplotlib style for technical drawings."""
        text_size = 4.0  # mm (font height)
        tick_size = 0.2  # mm

        # Convert mm to typographic points (1 inch = 25.4 mm = 72 points)
        mm_to_points = 72 / 25.4
        self.points_text = text_size * mm_to_points
        self.points_ticks = tick_size * mm_to_points

        plt.rcParams.update(
            {
                "font.family": "sans-serif",
                "font.size": self.points_text,
                "xtick.major.size": self.points_ticks,
                "ytick.major.size": self.points_ticks,
                "xtick.minor.size": self.points_ticks,
                "ytick.minor.size": self.points_ticks,
                "grid.linewidth": self.points_ticks,
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

    def _save_figure_exact_size(self, fig, ax, filename: str):
        """
        Resizes the figure so that 1 unit in the plot equals exactly 1 mm in reality,
        then saves it to filename.

        This method calculates the current display PPI and resizes the figure content
        to match the target physical dimensions defined by the axes data. It then
        saves the file with `bbox_inches='tight'` and a small pad to include external labels.

        Args:
            fig (matplotlib.figure.Figure): The figure object.
            ax (matplotlib.axes.Axes): The axes object containing the plot data.
            filename (str): The output filename (e.g., 'test.svg').
        """
        try:
            # 1. Setup temporary DPI for calculation
            calc_dpi = 100
            fig.set_dpi(calc_dpi)
            fig.canvas.draw()

            # 2. Get current pixels per unit
            p0 = ax.transData.transform((0, 0))
            p1 = ax.transData.transform((1, 1))

            current_x_pix_per_unit = abs(p1[0] - p0[0])

            # 3. Calculate Scale
            # We want 1 unit = 1 mm = 1/25.4 inches.
            scale_x = (1 / 25.4) / (current_x_pix_per_unit / calc_dpi)

            # 4. Apply Scaling
            curr_w, curr_h = fig.get_size_inches()
            # We assume square pixels (x scale == y scale)
            fig.set_size_inches(curr_w * scale_x, curr_h * scale_x)

            final_dpi = round(self.pix_per_mm * 25.4)
            fig.set_dpi(final_dpi)

            # 5. Verification
            fig.canvas.draw()
            p0_new = ax.transData.transform((0, 0))
            p1_new = ax.transData.transform((1, 1))
            new_x_pix = abs(p1_new[0] - p0_new[0])

            target_pix_per_unit = final_dpi / 25.4

            # Verify scaling accuracy
            if abs(new_x_pix - target_pix_per_unit) > 1.0:
                logger.warning(
                    f"Scale mismatch in {filename}! "
                    f"Expected {target_pix_per_unit:.1f} px/mm, got {new_x_pix:.1f} px/mm"
                )

            # 6. Save
            # Calculate full image size including labels
            renderer = fig.canvas.get_renderer()
            bbox = fig.get_tightbbox(renderer)

            full_w_mm = bbox.width * 25.4
            full_h_mm = bbox.height * 25.4

            output_path = os.path.join(self.script_directory, filename)
            logger.info(
                f"Saving {filename} (Image Size: {full_w_mm:.1f}x{full_h_mm:.1f}mm)..."
            )
            plt.savefig(output_path, bbox_inches="tight", dpi=final_dpi, pad_inches=0)

        except Exception as e:
            logger.error(f"Failed to save {filename}: {e}")
            raise
        finally:
            plt.close(fig)

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

    def generate_fan_test(self):
        """
        Generates a radial 'Fan' test pattern.

        This pattern consists of triangles rotated around a center point to form
        a starburst shape. It is useful for checking resolution at varying angles
        and determining the minimum resolvable feature size (inner circle limit).
        """
        logger.info("Generating Fan Test (Compensated)...")
        final_linewidth_target = 0.300  # mm (max width)
        pattern_radius = 30  # mm

        # Compensate the width
        # We define the triangle by its widest point (outer edge).
        comp_final_width = self._get_compensated_width(final_linewidth_target)

        # If the laser is wider than the line, we can't draw the start.
        if comp_final_width <= 0:
            logger.warning(
                "Fan test: Laser spot is larger than max linewidth! Skipping."
            )
            return

        # Triangle definition (using compensated width)
        triangle_vertices = np.array(
            [
                [0, 0],
                [-comp_final_width / 2, pattern_radius],
                [comp_final_width / 2, pattern_radius],
            ]
        )

        fig, ax = plt.subplots()
        ax.set_aspect("equal")

        # Angle depends on TARGET width (the pitch), not compensated width
        # This ensures the frequency of the starburst matches the physical design intent
        wedge_angle = np.arctan(final_linewidth_target / (pattern_radius * 2)) * 2
        num_triangles = int((0.5 * np.pi) / (wedge_angle * 2) + 1)

        for i in range(1, num_triangles):
            angle = -2 * i * wedge_angle
            t = Affine2D().rotate(angle)
            rotated_verts = t.transform(triangle_vertices)
            ax.add_patch(
                Polygon(rotated_verts, closed=True, facecolor="black", edgecolor=None)
            )

        ax.set_xlim(0, pattern_radius)
        ax.set_ylim(0, pattern_radius)

        # White circle to mask the messy center
        # We calculate the point where the physical lines would merge
        # This occurs when arc_length ~= laser_spot_mm
        r_start = (pattern_radius / final_linewidth_target) * self.laser_spot_mm
        ax.add_patch(
            Circle((0, 0), r_start, facecolor="white", edgecolor="white", zorder=10)
        )

        ticks = np.arange(0, pattern_radius + 1, 10)
        labels = [f"{round(x * 10)}" for x in ticks]

        for tick in np.arange(0, pattern_radius + 1, 5):
            ax.add_patch(
                Circle(
                    (0, 0), tick, fill=False, edgecolor="white", linewidth=0.5, zorder=5
                )
            )

        ax.set_xticks(ticks)
        ax.set_xticklabels(labels)
        ax.set_yticks(ticks)
        ax.set_yticklabels(labels)

        self._clean_axes(ax, "Radius [mm]", "Radius [mm]")
        self._save_figure_exact_size(fig, ax, "fantest.svg")

    def generate_wedge_jitter_test(self):
        """
        Generates a traditional wedge-based jitter test.

        Creates a series of vertical triangular strips. Inconsistencies in the
        width of these strips during printing indicate timing jitter in the
        fast-scan axis.
        """
        logger.info("Generating Wedge Jitter Test (Compensated)...")
        pattern_x = 25.0
        pattern_y = 30.0
        max_linewidth_target = 0.300
        min_linewidth_target = 0.05  # Note: Might be unprintable at tip if < spot size

        fig, ax = plt.subplots()

        # How many triangles fit? Based on TARGET width (pitch)
        num_triangles = int(pattern_x // max_linewidth_target)

        # Digital width calculation
        comp_max_width = self._get_compensated_width(max_linewidth_target)

        for i in range(1, num_triangles):
            cx = i * max_linewidth_target  # Center remains constant (based on pitch)

            if i % 2 == 0 and comp_max_width > 0:
                # Triangle geometry adjusted for spot size
                # We simply shrink the width, maintaining the center line
                verts = np.array(
                    [
                        [cx - comp_max_width / 2, pattern_y],
                        [cx, 0],  # Tip converges to center
                        [cx + comp_max_width / 2, pattern_y],
                    ]
                )
                ax.add_patch(
                    Polygon(verts, closed=True, facecolor="black", edgecolor="none")
                )

        # Calculate where the line becomes unprintable (width < spot size)
        y_start = (pattern_y / max_linewidth_target) * self.laser_spot_mm

        ax.set_xlim(0, pattern_x)
        ax.set_ylim(y_start, pattern_y)

        y_ticks = np.arange(y_start, pattern_y + 1, 5)
        ax.set_yticks(y_ticks)
        ax.grid(axis="y", color="white", linestyle="-")

        self._clean_axes(ax, "Offset [mm]", "Height [mm]")
        self._save_figure_exact_size(fig, ax, "jitter_wedge.svg")

    def generate_combined_test(self):
        """
        Generates a Combined Grid Test for Jitter and Cross-Scan errors.

        This creates a single square grid (default 30x30mm) containing:
        1. Vertical Lines (Columns): Varying thickness to test X-axis (fast) jitter.
        2. Horizontal Lines (Rows): Varying thickness to test Y-axis (slow) cross-scan error.

        Labels are placed on the outside of the grid, and a minimal layout
        is used (no grid lines, only boundary ticks) to prevent visual clutter.
        """
        logger.info("Generating Combined Grid Test (Compensated)...")
        box_size = 30  # mm (Square size)
        cell_size = 5  # mm (Size of each thickness zone)
        linewidth_start = 0.04  # mm
        stepsize = 0.020  # mm
        lines_per_group = 10  # Number of lines per grid cell

        fig, ax = plt.subplots()

        def get_thickness(n):
            return n * stepsize + linewidth_start

        num_steps = int(box_size // cell_size)
        tick_locs = []
        tick_labels = []

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

        ax.set_xlabel("fast axis")
        ax.set_ylabel("slow axis")
        # Added spot size to title for documentation on the print itself
        ax.set_title(
            "linewidth [μm]",
            loc="left",
            x=-0.2,
        )

        ax.set_xlim(0, box_size)
        ax.set_ylim(0, box_size)

        for key, spine in ax.spines.items():
            spine.set_visible(False)

        ax.grid(False)
        ax.tick_params(axis="both", which="major", length=5, width=1)
        ax.tick_params(axis="both", which="minor", length=0)

        self._save_figure_exact_size(fig, ax, "combined_grid_test.svg")


if __name__ == "__main__":
    try:
        generator = LaserCalibrationGen(laser_spot_size_microns=30.0)

        # Generate all patterns
        generator.generate_wedge_jitter_test()
        generator.generate_fan_test()
        generator.generate_combined_test()

        logger.info("All patterns generated successfully.")

    except Exception as e:
        logger.critical(f"An unexpected error occurred: {e}")
        sys.exit(1)
