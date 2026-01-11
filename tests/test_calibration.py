import pytest
import os
from hexastorm.calibration import run_full_calibration_analysis

# 1. Define Data
EXPECTED_OUTPUT = {
    "0": {
        "x_shift_px": 0.0,
        "y_shift_px": 0.0,
        "std_x": 0.0,
        "mean_spot_size_um": 29.588,
        "mean_eccentricity": 1.887,
    },
    "1": {
        "x_shift_px": -9.146,
        "y_shift_px": -28.014,
        "std_x": 0.468,
        "mean_spot_size_um": 28.775,
        "mean_eccentricity": 1.77,
    },
    "2": {
        "x_shift_px": 3.998,
        "y_shift_px": -23.814,
        "std_x": 1.749,
        "mean_spot_size_um": 26.467,
        "mean_eccentricity": 1.763,
    },
    "3": {
        "x_shift_px": -7.879,
        "y_shift_px": -27.756,
        "std_x": 0.669,
        "mean_spot_size_um": 26.496,
        "mean_eccentricity": 1.83,
    },
}


# 2. FIXTURE: Run the heavy calculation only ONCE
# scope="module" means: run this function once per file, not once per test.
@pytest.fixture(scope="module")
def calibration_result():
    test_dir = os.path.dirname(__file__)
    image_folder = os.path.join(test_dir, "data")

    # Return the dictionary of results to be used by tests
    return run_full_calibration_analysis(image_folder, debug=False)


# 3. PARAMETRIZE: Generate a separate test case for every item in EXPECTED_OUTPUT
@pytest.mark.parametrize("facet_id, expected_metrics", EXPECTED_OUTPUT.items())
def test_facet_accuracy(calibration_result, facet_id, expected_metrics):
    """
    This function will run 4 times (once for each facet).
    If Facet 0 fails, Facet 1, 2, and 3 still run!
    """
    # Check if the facet exists in the output
    assert facet_id in calibration_result, f"Facet {facet_id} missing from output"

    actual_metrics = calibration_result[facet_id]

    # Check all metrics for this specific facet
    for metric_name, expected_val in expected_metrics.items():
        actual_val = actual_metrics.get(metric_name)

        assert actual_val is not None, (
            f"Metric {metric_name} missing for Facet {facet_id}"
        )

        # Compare
        assert actual_val == pytest.approx(expected_val, rel=1e-3), (
            f"Facet {facet_id} mismatch on {metric_name}"
        )
