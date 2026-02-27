"""
Run test:
    uv run pytest tests/test_stacker.py -v
"""

import json
import logging
from unittest.mock import patch, MagicMock

import pytest
import cv2 as cv
import numpy as np

from hexastorm.calibration.stack import LaserStackSimulator

logger = logging.getLogger(__name__)

# ==============================================================================
# SYNTHETIC DATA GENERATOR
# ==============================================================================


def generate_vertical_dot_line(img_w=500, img_h=500):
    """Generates a black image with a single vertical line of 5 dots."""
    img = np.zeros((img_h, img_w), dtype=np.uint8)

    # Place 5 dots vertically in the center of the image
    center_x = img_w // 2
    y_positions = [100, 175, 250, 325, 400]

    for y in y_positions:
        cv.circle(img, (center_x, y), 5, 255, -1)

    return img


# ==============================================================================
# THE TEST
# ==============================================================================
#
# This test mathematically proves the affine translation and image accumulator
# logic of the LaserStackSimulator without relying on real hardware configs.
#
# 1. ISOLATION (Mocking): We use @patch to intercept file reads and configs.
#    We force the calibration history to return an angle of -90 degrees
#    (which neutralizes the stacker's +90 degree offset, keeping our test
#    image perfectly vertical). We also force a clean step size of 50.0 px.
#
# 2. SYNTHETIC INPUT: We generate a pure black canvas with a single, perfectly
#    vertical line of 5 white dots.
#
# 3. EXECUTION: We simulate 3 movement steps. The stacker should stamp the
#    5-dot line onto the master canvas 3 times, shifting it 50px on the X-axis.
#
# 4. VERIFICATION: We ask OpenCV to analyze the final stacked image. We assert:
#    - Exactly 15 dots exist (5 dots * 3 steps).
#    - The dots form exactly 3 unique vertical columns.
#    - The distance between each column is exactly 50 pixels.


@patch("hexastorm.calibration.stack.PlatformConfig")  # Mock the config
@patch("builtins.open", new_callable=MagicMock)  # Mock the JSON file read
def test_laser_stack_simulator(mock_open, mock_config_class):
    # 1. Force the JSON read to return an angle of -90.0
    # Because your code does: rot_angle = mean_angle_deg + 90
    # -90 + 90 = 0 degrees of rotation (keeps our vertical line vertical)
    mock_history = [{"facets": {"0": {"rotation_angle": -90.0}}}]
    mock_open.return_value.__enter__.return_value.read.return_value = json.dumps(
        mock_history
    )

    # 2. Force the Config to yield a predictable step size
    mock_cfg_instance = mock_config_class.return_value
    mock_cfg_instance.paths = {"calibration": MagicMock()}  # Fake Path object

    # Let's say we want a step of exactly 50 pixels.
    # Your code: self.step_px = (1 / steps_mm) * 1e3 / px_size_um
    # We will just patch `self.step_px` directly after init to bypass the deep motor dict lookups

    # 3. Initialize Stacker
    EXPECTED_STEPS = 3
    FORCED_STEP_PX = 50.0

    stacker = LaserStackSimulator(expected_total_steps=EXPECTED_STEPS)
    stacker.step_px = FORCED_STEP_PX  # Override with our clean test value

    # 4. Run the simulation
    test_img = generate_vertical_dot_line()

    for step_idx in range(EXPECTED_STEPS):
        stacker.add_exposures(test_img, num_steps=step_idx)

    # 5. Get Result
    final_img = stacker.get_result()
    assert final_img is not None, "Stacker returned None!"

    # 6. Verify the math (Using standard OpenCV contour finding)
    # We stacked a line of 5 dots, 3 times -> We should have 15 dots total.
    _, thresh = cv.threshold(final_img, 127, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    assert len(contours) == 15, f"Expected 15 dots, found {len(contours)}"

    # Extract X coordinates of all dots
    x_coords = []
    for cnt in contours:
        M = cv.moments(cnt)
        if M["m00"] > 0:
            cx = M["m10"] / M["m00"]
            x_coords.append(cx)

    # Group the X coordinates by column (rounding to nearest pixel)
    unique_columns = np.unique(np.round(x_coords))

    # We expect exactly 3 columns of dots
    assert len(unique_columns) == 3, f"Expected 3 columns, found {len(unique_columns)}"

    # Check the spatial distance between column 0 -> 1, and 1 -> 2
    # It should be exactly equal to our FORCED_STEP_PX (50 pixels)
    dist_1 = unique_columns[1] - unique_columns[0]
    dist_2 = unique_columns[2] - unique_columns[1]

    assert dist_1 == pytest.approx(FORCED_STEP_PX, abs=1.0), (
        f"Step 1 distance was {dist_1}, expected {FORCED_STEP_PX}"
    )
    assert dist_2 == pytest.approx(FORCED_STEP_PX, abs=1.0), (
        f"Step 2 distance was {dist_2}, expected {FORCED_STEP_PX}"
    )

    logger.info("Stacker verified: Images translated exactly 50 pixels per step.")
