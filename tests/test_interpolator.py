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
    """
    exptected_hash = "6805fdfd71605f5e1ac8b68244430aaa"
    # Setup paths
    fname = "jittertest"
    # Assuming the SVG is in the standard location relative to the test
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

    # 2. Write to a temporary file (so we don't clutter disk)
    out_file = tmp_path / f"{fname}.bin"
    interpolator.writebin(ptrn, str(out_file))

    # 3. Verify Output
    # Check 1: File exists and has size
    assert out_file.exists()
    assert out_file.stat().st_size > 0

    # Check 2: Content Hash (The Golden Master Check)
    with open(out_file, "rb") as f:
        file_hash = hashlib.md5(f.read()).hexdigest()

    # Fail if the hash changed (meaning logic changed)
    assert file_hash == exptected_hash, (
        f"Logic changed! Expected {exptected_hash}, got {file_hash}"
    )


def test_roundtrip_integrity(interpolator, tmp_path):
    """
    Checks if Write -> Read produces the same parameters.
    This validates your readbin/writebin symmetry.
    """
    # Stel handmatig neppe afmetingen in (Mock Geometry)
    # Normaal doet 'patternfile' dit voor je.
    interpolator.params["samplexsize"] = 50.0  # mm
    interpolator.params["sampleysize"] = 50.0  # mm
    interpolator.params["facetsinlane"] = 100  # Aantal facetten per baan

    # Haal de berekende parameters op
    params = interpolator.params
    lanes = int(np.ceil(params["samplexsize"] / params["lanewidth"]))
    facets = int(params["facetsinlane"])
    bytes_line = int(np.ceil(params["bitsinscanline"] / 8))

    # 2. Create deterministic random data
    rng = np.random.default_rng(42)
    # Nu is size > 0, dus dit werkt!
    dummy_data = rng.integers(0, 255, size=lanes * facets * bytes_line, dtype=np.uint8)

    # 3. Write
    out_file = tmp_path / "test_roundtrip.bin"
    interpolator.writebin(dummy_data, out_file)

    # 4. Read back
    # Let op: readbin verwacht alleen de bestandsnaam in debug folder
    r_facets, r_lanes, r_width, r_data = interpolator.readbin(out_file)

    # 5. Verify
    assert r_facets == facets
    assert r_lanes == lanes
    assert np.allclose(r_width, params["lanewidth"])

    # Check of data gelijk is (kan lastig zijn door padding bits aan het eind van regels)
    # Maar voor byte-aligned data zou dit moeten werken:
    assert len(r_data) == len(dummy_data)
    assert np.array_equal(r_data, dummy_data)
