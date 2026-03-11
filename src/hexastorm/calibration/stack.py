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
        px_size_um=Camera.DEFAULT_PIXEL_SIZE_UM,  # Placeholder: Replace with
    ):
        self.cfg = PlatformConfig(test=False)

        history_file = self.cfg.paths["calibration"] / "calibration_history.json"
        with open(history_file, "r") as f:
            history = json.load(f)
            last_record = history[-1]
            facets = last_record["facets"]

        angles = [facets[facet]["rotation_angle"] for facet in facets.keys()]

        self.expected_total_steps = expected_total_steps

        # 1. Calculate the rotation needed to make the line parallel to Y-axis
        # If your angles are around -86 degrees, rotating by +6 degrees (or 90 + angle)
        # aligns it. We use the mean angle directly for the rotation matrix.
        self.mean_angle_deg = np.mean(angles)

        # 2. Movement is now purely horizontal (X) in the rotated frame
        mtr_cfg = self.cfg.motor_cfg
        steps_mm = mtr_cfg["steps_mm"][mtr_cfg["orth2lsrline"]]
        self.step_px = (1 / steps_mm) * 1e3 / px_size_um

        # State variables
        self.current_step = 0
        self.accumulator = None
        self.rotation_matrix = None
        self.canvas_offset_x = 0

        # pad margin final crop
        self.pad = 15  # pixels

    def _prepare_transforms(self, h, w):
        """Calculates rotation and pre-allocates canvas."""
        # Center of the source image for rotation
        center = (w // 2, h // 2)

        # We rotate by (mean_angle + 90) to bring the line to 0 degrees (vertical)
        # Adjust the +90 based on whether your 'angle' is relative to X or Y.
        # Based on your description, rotating by -mean_angle - 90 aligns to Y.
        rot_angle = self.mean_angle_deg + 90

        # Get rotation matrix
        self.rotation_matrix = cv.getRotationMatrix2D(center, rot_angle, 1.0)

        # Calculate canvas size: Original width + total movement
        total_move_px = abs(self.step_px * self.expected_total_steps)
        canvas_w = int(w + total_move_px) + 50
        canvas_h = h + 50  # Adding a small buffer

        # If movement is negative, we need an offset so we don't draw off-screen
        self.canvas_offset_x = 0 if self.step_px > 0 else total_move_px

        self.accumulator = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    def add_exposures(self, img, num_steps=1):
        h, w = img.shape[:2]

        # otsu binarization, check for blankimage
        if np.max(img) < 50:
            clean_img = np.zeros_like(img)
        else:
            _, clean_img = cv.threshold(img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        if self.accumulator is None:
            self._prepare_transforms(h, w)

        # Step 1: Rotate the source image so the line is vertical
        rotated_img = cv.warpAffine(
            clean_img, self.rotation_matrix, (w, h), flags=cv.INTER_LINEAR
        )

        # Step 2: Calculate pure X shift
        shift_x = self.canvas_offset_x + (num_steps * self.step_px)
        shift_y = 25  # Small constant Y buffer

        # Simple translation matrix
        M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])

        # Step 3: Warp onto master canvas
        shifted_img = cv.warpAffine(
            rotated_img,
            M,
            (self.accumulator.shape[1], self.accumulator.shape[0]),
            flags=cv.INTER_LINEAR,
        )

        # np.maximum prevents accumulating background noise
        # it is now also prevented by otsu thresholding
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
    current_img = images[0]

    # 2. Initialize the Simulator
    # We are doing 10 loops of 10 steps each = 100 total steps
    stacker = LaserStackSimulator(expected_total_steps=100)

    # 3. Process the stack
    # Example: alternating through your facet images
    for step in range(100):
        # Add it to the canvas 10 times, updating the continuous offset
        stacker.add_exposures(current_img, num_steps=step)

    # 4. Extract and save
    final_stack = stacker.get_result()
    cv.imwrite("final_calibration_grid.jpg", final_stack)
    logger.info("execution complete, final image saved as final_calibration_grid.jpg")
