"""
Run test:
    uv run pytest tests/test_calibration.py -v
"""

import pytest
import cv2 as cv
import numpy as np
from pathlib import Path
import logging

from hexastorm.calibration.analyzer import calibration
from hexastorm.config import Camera
from hexastorm.log_setup import configure_logging

logger = logging.getLogger(__name__)

# ==============================================================================
# CONFIGURATION & GROUND TRUTH
# ==============================================================================

PIXEL_SIZE_UM = Camera.DEFAULT_PIXEL_SIZE_UM
DEBUG = False  # Visual debug for 1D line calibration test

# --- 1D Line Configuration ---
TARGET_ANGLE_DEG = -86.0
SYNTHETIC_SHIFTS_UM = [
    {"scan": -30.0, "orth": 20.0},
    {"scan": 10.0, "orth": -10.0},
    {"scan": 40.0, "orth": -30.0},
    {"scan": -20.0, "orth": 20.0},
]

# --- 2D Grid Configuration ---
GRID_ROWS = 10
GRID_COLS = 10
SPACING_UM = 300.0
DOT_DIAMETER_UM = 100.0
GRID_SHIFTS_UM = [
    {"scan": 0.0, "orth": 0.0},  # Facet 0 (Reference)
    {"scan": 15.0, "orth": -20.0},  # Facet 1
    {"scan": -12.5, "orth": 35.0},  # Facet 2
    {"scan": 40.0, "orth": 8.0},  # Facet 3
]

# ==============================================================================
# PART 1: 1D LINE CALIBRATION TEST
#
# The system is tested by creating images of 5 dots in a line for each facet
# Each facet has a known shift in scan and orthogonal direction and a known
# rotation
# ==============================================================================


def generate_synthetic_facets(output_dir: Path):
    img_w, img_h = 1000, 1000
    angle_rad = np.radians(TARGET_ANGLE_DEG)

    base_x = np.array([-200, -100, 0, 100, 200], dtype=float)
    base_y = np.zeros_like(base_x)

    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    rot_mat = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    ground_truth = {}
    px_size = abs(PIXEL_SIZE_UM)

    for i, shifts in enumerate(SYNTHETIC_SHIFTS_UM):
        scan_um = shifts["scan"]
        orth_um = shifts["orth"]

        scan_px = scan_um / px_size
        orth_px = orth_um / px_size

        x_shifted = base_x + scan_px
        y_shifted = base_y + orth_px
        pts = np.column_stack((x_shifted, y_shifted))

        pts_tilted = pts @ rot_mat.T

        pts_tilted[:, 0] += img_w / 2
        pts_tilted[:, 1] += img_h / 2

        img = np.zeros((img_h, img_w, 3), dtype=np.uint8)
        dot_radius_px = 6

        for pt in pts_tilted:
            cv.circle(
                img,
                (int(round(pt[0])), int(round(pt[1]))),
                dot_radius_px,
                (255, 255, 255),
                -1,
            )

        cv.imwrite(str(output_dir / f"facet{i}.jpg"), img)

        ground_truth[str(i)] = {
            "Scan_shift_um": scan_um,
            "Orth_shift_um": orth_um,
            "rotation_angle_deg": TARGET_ANGLE_DEG,
        }

    return ground_truth


@pytest.fixture
def generated_data(tmp_path):
    configure_logging(logging.ERROR)
    img_dir = tmp_path / "images_1d"
    img_dir.mkdir()
    ground_truth = generate_synthetic_facets(img_dir)
    return img_dir, ground_truth


def test_calibration_accuracy(generated_data):
    img_dir, ground_truth = generated_data

    results = calibration(image_dir=img_dir, num_facets=4, debug=DEBUG, store_log=False)
    assert results, "Calibration returned empty results!"

    for facet_id, expected_metrics in ground_truth.items():
        assert facet_id in results, f"Facet {facet_id} missing from output"
        actual_metrics = results[facet_id]

        actual_angle = actual_metrics.get("rotation_angle_deg")
        assert actual_angle == pytest.approx(
            expected_metrics["rotation_angle_deg"], abs=0.1
        )

        actual_scan = actual_metrics.get("Scan_shift_um")
        assert actual_scan == pytest.approx(expected_metrics["Scan_shift_um"], abs=2.0)

        actual_orth = actual_metrics.get("Orth_shift_um")
        assert actual_orth == pytest.approx(expected_metrics["Orth_shift_um"], abs=2.0)


# ==============================================================================
# PART 2: 2D GRID VERIFICATION TEST
#
# The actual system is tested by creating images of a 10 x 10 grid for each facet
# These images are artifically stiched by the stacker and are than expected
# to align. The advantage of this is that we can test the entire system (slicer
# and laser head). Below is an artificailly generated grid image with known shifts
# in scan and orthogonal direction.
# ==============================================================================


def generate_2d_grid_image(output_path: Path, shift_x_um: float, shift_y_um: float):
    img_w, img_h = 1000, 1000
    img = np.zeros((img_h, img_w, 3), dtype=np.uint8)

    px_size = abs(PIXEL_SIZE_UM)
    dot_radius_px = (DOT_DIAMETER_UM / px_size) / 2.0
    spacing_px = SPACING_UM / px_size
    shift_x_px = shift_x_um / px_size
    shift_y_px = shift_y_um / px_size

    start_x = (img_w - (GRID_COLS - 1) * spacing_px) / 2
    start_y = (img_h - (GRID_ROWS - 1) * spacing_px) / 2

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x = start_x + (col * spacing_px) + shift_x_px
            y = start_y + (row * spacing_px) + shift_y_px

            cv.circle(
                img,
                (int(round(x)), int(round(y))),
                int(round(dot_radius_px)),
                (255, 255, 255),
                -1,
            )

    cv.imwrite(str(output_path), img)


def test_2d_verification(tmp_path):
    img_dir = tmp_path / "images_2d"
    img_dir.mkdir()

    # 1. Generate all 4 facet grid images using the synthetic generator
    for i in range(4):
        target_path = img_dir / f"facet{i}.jpg"
        generate_2d_grid_image(
            target_path, GRID_SHIFTS_UM[i]["scan"], GRID_SHIFTS_UM[i]["orth"]
        )

    # 2. Run the unified calibration function in 2D mode
    results = calibration(
        image_dir=img_dir,
        num_facets=4,
        debug=False,
        store_log=False,
        compute_rotation=False,  # Tells analyzer to treat this as a 2D grid
    )

    assert results, "2D Calibration returned empty results!"

    # 3. Calculate the expected virtual reference (the mean of all input shifts)
    mean_scan = sum(f["scan"] for f in GRID_SHIFTS_UM) / len(GRID_SHIFTS_UM)
    mean_orth = sum(f["orth"] for f in GRID_SHIFTS_UM) / len(GRID_SHIFTS_UM)

    # 4. Verify each facet against its expected RELATIVE shift
    for i in range(4):
        facet_key = str(i)
        assert facet_key in results, f"Facet {i} missing from output"

        # The true deviation is the absolute position minus the group center
        expected_scan_rel = GRID_SHIFTS_UM[i]["scan"] - mean_scan
        expected_orth_rel = GRID_SHIFTS_UM[i]["orth"] - mean_orth

        actual_scan = results[facet_key].get("Scan_shift_um")
        actual_orth = results[facet_key].get("Orth_shift_um")
        actual_angle = results[facet_key].get("rotation_angle_deg")

        logger.info(
            f"Facet {i} 2D - Expected Rel: Scan={expected_scan_rel:.2f}, Orth={expected_orth_rel:.2f} | "
            f"Calculated: Scan={actual_scan:.2f}, Orth={actual_orth:.2f}"
        )

        assert actual_scan == pytest.approx(expected_scan_rel, abs=1.5)
        assert actual_orth == pytest.approx(expected_orth_rel, abs=1.5)

        # Ensure rotation is forced to 0.0 in 2D mode
        assert actual_angle == 0.0
