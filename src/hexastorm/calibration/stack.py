import logging
import json
from pathlib import Path

import cv2 as cv
import numpy as np

from ..config import PlatformConfig, Camera

logger = logging.getLogger(__name__)


class LaserStackSimulator:
    def __init__(
        self,
        expected_total_steps,
        exposures=1,
        px_size_um=Camera.DEFAULT_PIXEL_SIZE_UM,
        rotate=True,
    ):
        """
        Simulates the 2D physical exposure of a polygon laser scanhead by stacking
        camera captures by shifting the line in the direction of movement, i.e. ortoghonal
        to the scanline.

        Args:
            expected_total_steps (int): The total number of steps expected to compute canvas size
            exposures (int): Number of exposure per step, in reality multiple exposure per line are needed
            px_size_um (float): Physical size of a camera pixel in micrometers.
            rotate (bool): Toggles the stacking algorithm.
                - True (Default): Rotates the raw image to perfectly align the laser
                line with the horizontal axis prior to stacking. This results in a
                clean, square final canvas but mathematically interpolates the image,
                which may slightly distort raw pixel centroids.
                - False: Uses vector math to shift the raw, unrotated image along the
                measured angular trajectory. This preserves exact raw pixel data
                but results in a diagonally skewed bounding box.
        """
        self.cfg = PlatformConfig(test=False)

        history_file = self.cfg.paths["calibration"] / "calibration_history.json"
        with open(history_file, "r") as f:
            history = json.load(f)
            last_record = history[-1]
            facets = last_record["facets"]

        angles = [facets[facet]["rotation_angle"] for facet in facets.keys()]

        self.expected_total_steps = expected_total_steps
        self.rotate = rotate

        # 1. Calculate the rotation needed to make the line parallel to Y-axis
        self.mean_angle_deg = np.mean(angles)

        # 2. Movement is now purely horizontal (X) in the rotated frame
        mtr_cfg = self.cfg.motor_cfg
        steps_mm = mtr_cfg["steps_mm"][mtr_cfg["orth2lsrline"]]
        self.step_px = (1 / steps_mm) * 1e3 / (px_size_um * exposures)

        # State variables
        self.current_step = 0
        self.accumulator = None
        self.rotation_matrix = None
        self.canvas_offset_x = 0
        self.canvas_offset_y = 0
        self.step_dx = 0
        self.step_dy = 0

        # pad margin final crop
        self.pad = 15  # pixels

    def _prepare_transforms(self, h, w):
        """Prepares canvas and calculates transforms."""
        if not self.rotate:
            # --- VECTOR ALGORITHM ---
            # Convert angle to radians.
            angle_rad = np.radians(self.mean_angle_deg)

            # Calculate how many pixels we move in X and Y for a single step
            self.step_dx = -np.sin(angle_rad) * self.step_px
            self.step_dy = np.cos(angle_rad) * self.step_px

            # Calculate total canvas expansion
            total_move_x = abs(self.step_dx * self.expected_total_steps)
            total_move_y = abs(self.step_dy * self.expected_total_steps)

            canvas_w = int(w + total_move_x + 100)
            canvas_h = int(h + total_move_y + 100)

            # Offsets to ensure we never draw into negative pixel space
            self.canvas_offset_x = 50 if self.step_dx > 0 else int(total_move_x) + 50
            self.canvas_offset_y = 50 if self.step_dy > 0 else int(total_move_y) + 50

            self.accumulator = np.zeros((canvas_h, canvas_w), dtype=np.float32)

        else:
            # --- ROTATION ALGORITHM ---
            center = (w // 2, h // 2)

            # To flatten the line to 0 degrees (horizontal), rotate by the inverse angle.
            rot_angle = self.mean_angle_deg
            self.rotation_matrix = cv.getRotationMatrix2D(center, rot_angle, 1.0)

            # Canvas grows in Height (Y), not Width (X)
            total_move_px = abs(self.step_px * self.expected_total_steps)
            canvas_w = w + 100
            canvas_h = int(h + total_move_px) + 200

            # Offset applied to Y to prevent drawing off-screen
            self.canvas_offset_y = 0 if self.step_px > 0 else total_move_px

            self.accumulator = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    def add_exposures(self, img, num_steps=1):
        """Adds image to accumalator by taking into account the number of steps

        Args:
            img:  image to add
            num_steps: number of steps to shift the image
        """
        h, w = img.shape[:2]

        # otsu binarization, check for blankimage
        if np.max(img) < 50:
            clean_img = np.zeros_like(img)
        else:
            _, clean_img = cv.threshold(img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        if self.accumulator is None:
            self._prepare_transforms(h, w)

        if not self.rotate:
            # Calculate the exact X and Y position for this step along the vector
            shift_x = self.canvas_offset_x + (num_steps * self.step_dx)
            shift_y = self.canvas_offset_y + (num_steps * self.step_dy)

            # Pure translation, zero rotation.
            M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])

            shifted_img = cv.warpAffine(
                clean_img,
                M,
                (self.accumulator.shape[1], self.accumulator.shape[0]),
                flags=cv.INTER_LINEAR,
            )

        else:
            # Step 1: Rotate the source image so the line is horizontal
            rotated_img = cv.warpAffine(
                clean_img, self.rotation_matrix, (w, h), flags=cv.INTER_LINEAR
            )

            # Step 2: Calculate pure Y shift (Stage stepping)
            shift_x = 50  # Small constant X buffer
            shift_y = self.canvas_offset_y + (num_steps * self.step_px) + 100

            # Translation matrix
            M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])

            # Step 3: Warp onto master canvas
            shifted_img = cv.warpAffine(
                rotated_img,
                M,
                (self.accumulator.shape[1], self.accumulator.shape[0]),
                flags=cv.INTER_LINEAR,
            )

        # np.maximum prevents accumulating background noise
        self.accumulator = np.maximum(self.accumulator, shifted_img)
        self.current_step += 1

    def get_result(self):
        """Normalizes and returns the fixed-size canvas to preserve absolute spatial coordinates."""
        if self.accumulator is None:
            return None

        acc = self.accumulator

        # Normalize the full, uncropped accumulator to 0-255
        final_min, final_max = np.min(acc), np.max(acc)

        # Guard against a completely blank canvas
        if final_max == final_min:
            return np.zeros_like(acc, dtype=np.uint8)

        normalized = (acc - final_min) / (final_max - final_min) * 255.0

        logger.debug(f"Final Canvas Size: {normalized.shape[1]}x{normalized.shape[0]}")
        return normalized.astype(np.uint8)


if __name__ == "__main__":
    from ..log_setup import configure_logging

    configure_logging()
    cfg = PlatformConfig(test=False)

    # 1. Setup Data
    filenames = [f"scan_error_facet_{facet}.jpg" for facet in range(4)]

    # Load images once
    images = []
    for fn in filenames:
        img_path = Path(cfg.paths["images"]) / fn
        img = cv.imread(str(img_path), cv.IMREAD_GRAYSCALE)
        if img is not None:
            images.append(img)

    if images:
        current_img = images[0]

        # 2. Initialize the Simulator (Set rotate=False to test the vector algorithm)
        stacker = LaserStackSimulator(expected_total_steps=100, rotate=False)

        # 3. Process the stack
        for step in range(100):
            stacker.add_exposures(current_img, num_steps=step)

        # 4. Extract and save
        final_stack = stacker.get_result()
        cv.imwrite("final_calibration_grid.jpg", final_stack)
        logger.info(
            "execution complete, final image saved as final_calibration_grid.jpg"
        )
    else:
        logger.error("No images found to process.")
