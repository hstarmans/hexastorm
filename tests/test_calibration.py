import pytest
from pathlib import Path
from hexastorm.calibration import run_full_calibration_analysis

# 1. Define Data
EXPECTED_OUTPUT = {
    "0": {
        "Scan_shift_um": 0.0,
        "Orth_shift_um": 0.0,
        "std_scan_um": 0.0,
        "mean_spot_size_um": 29.588,
        "mean_eccentricity": 1.887,
    },
    "1": {
        "Scan_shift_um": -27.438,
        "Orth_shift_um": -84.042,
        "std_scan_um": 1.404,
        "mean_spot_size_um": 28.775,
        "mean_eccentricity": 1.77,
    },
    "2": {
        "Scan_shift_um": 11.994,
        "Orth_shift_um": -71.442,
        "std_scan_um": 5.247,
        "mean_spot_size_um": 26.467,
        "mean_eccentricity": 1.763,
    },
    "3": {
        "Scan_shift_um": -23.637,
        "Orth_shift_um": -83.268,
        "std_scan_um": 2.007,
        "mean_spot_size_um": 26.496,
        "mean_eccentricity": 1.83,
    },
}


# 2. FIXTURE: Run the heavy calculation only ONCE
@pytest.fixture(scope="module")
def calibration_result():
    # Use pathlib for consistency with the rest of your project
    test_dir = Path(__file__).parent.resolve()
    image_folder = test_dir / "data"

    # Return the dictionary of results to be used by tests
    return run_full_calibration_analysis(image_dir=image_folder, debug=False)


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

        # Compare using approx to handle floating point fuzziness
        assert actual_val == pytest.approx(expected_val, rel=1e-3), (
            f"Facet {facet_id} mismatch on {metric_name}"
        )
