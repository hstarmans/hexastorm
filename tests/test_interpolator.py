import hashlib
from pathlib import Path

import numpy as np
import pytest

from hexastorm.interpolator.interpolator import Interpolator


@pytest.fixture
def interpolator():
    return Interpolator(stepsperline=1)


def test_regression_jittertest(interpolator, tmp_path):
    """
    Ensures that the entire pipeline (SVG -> Math -> Binary)
    produces the EXACT same bytes as the 'approved' version.

    NOTE: If you change the math or image processing logic,
    this test will fail. Validate the new output manually,
    then update the hash here.
    """
    # The hash of the "golden master" file.
    # NOTE: Since we fixed the white edge bug, this WILL fail the first time.
    # Run pytest, copy the 'got' hash from the error, and update this string.
    expected_hash = "fbd4611786c33a003e73333a83076fab"

    fname = "combined_grid_test"
    # Assuming the SVG is in the standard location relative to the test
    # Adjust this path if your tests folder is not at the project root
    project_root = Path(__file__).parent.parent
    svg_path = (
        project_root
        / "src"
        / "hexastorm"
        / "interpolator"
        / "patterns"
        / f"{fname}.svg"
    )

    if not svg_path.exists():
        pytest.fail(f"Kan patroon niet vinden op: {svg_path}")

    # 1. Run Pipeline
    ptrn = interpolator.patternfile(svg_path)

    # 2. Write to a temporary file
    out_file = tmp_path / f"{fname}.bin"
    interpolator.writebin(ptrn, out_file)

    # 3. Verify Output
    assert out_file.exists()
    assert out_file.stat().st_size > 0

    # Check Content Hash
    with open(out_file, "rb") as f:
        file_hash = hashlib.md5(f.read()).hexdigest()

    if file_hash != expected_hash:
        print(f"\nHash Mismatch! Logic changed.")
        print(f"Expected: {expected_hash}")
        print(f"Got:      {file_hash}")
        print(
            "If the changes are intentional (e.g. bug fix), update the test variable."
        )

    assert file_hash == expected_hash


def test_roundtrip_integrity(interpolator, tmp_path):
    """
    Checks if Write -> Read produces the same parameters.
    This validates the readbin/writebin symmetry and the new IO module.
    """
    # 1. Mock Geometry
    # We can write directly to the numba typed dict
    interpolator.params["samplexsize"] = 50.0  # mm
    interpolator.params["sampleysize"] = 50.0  # mm
    interpolator.params["facetsinlane"] = 100  # Aantal facetten per baan

    # Calculate expected sizes
    params = interpolator.params
    lanes = int(np.ceil(params["samplexsize"] / params["lanewidth"]))
    facets = int(params["facetsinlane"])
    bytes_line = int(np.ceil(params["bitsinscanline"] / 8))

    # 2. Create deterministic random data
    rng = np.random.default_rng(42)
    dummy_data = rng.integers(0, 255, size=lanes * facets * bytes_line, dtype=np.uint8)

    # 3. Write
    # The new writebin supports full paths, no need for .name hacks
    out_file = tmp_path / "test_roundtrip.bin"
    interpolator.writebin(dummy_data, out_file)

    # 4. Read back
    # The new readbin supports full paths too
    r_facets, r_lanes, r_width, r_data = interpolator.readbin(out_file)

    # 5. Verify
    assert r_facets == facets
    assert r_lanes == lanes
    assert np.allclose(r_width, params["lanewidth"])

    assert len(r_data) == len(dummy_data)
    assert np.array_equal(r_data, dummy_data)
