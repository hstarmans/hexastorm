import hashlib
from pathlib import Path

import numpy as np
import pytest

from hexastorm import ulabext
from hexastorm.exposure import io
from hexastorm.exposure.raster import RasterExposure


@pytest.fixture
def raster_exposure():
    return RasterExposure()


def test_regression_jittertest(raster_exposure, tmp_path):
    """
    Ensures that the entire pipeline (SVG -> Math -> Binary)
    produces the EXACT same bytes as the 'approved' version.

    NOTE: If you change the math or image processing logic,
    this test will fail. Validate the new output manually,
    then update the hash here.
    """
    # Golden-master hash for unified unrotated Substrate rasterization + non-dithered 1-bit conversion
    # (removed legacy -90 deg SVG rotation and directional Floyd-Steinberg dithering).
    expected_hash = "1c65e883b72bd3310e19ac09e530a02b"

    fname = "combined_grid_test"
    project_root = Path(__file__).resolve().parent.parent.parent
    svg_path = (
        project_root / "src" / "hexastorm" / "exposure" / "patterns" / f"{fname}.svg"
    )
    if not svg_path.exists():
        alt_svg_path = raster_exposure.cfg.paths["svgs"] / f"{fname}.svg"
        if not alt_svg_path.exists():
            from hexastorm.exposure.patterns.machine import LaserCalibrationGen

            LaserCalibrationGen(
                correction=False, exposures=1, mode="raster"
            ).generate_combined_test()
        svg_path = alt_svg_path

    # 1. Run Pipeline
    ptrn = raster_exposure.patternfile(svg_path)

    # 2. Write to a temporary file
    out_file = tmp_path / f"{fname}.bin"
    raster_exposure.writebin(ptrn, out_file)
    actual_out = raster_exposure._last_output_path

    # 3. Verify Output & Roundtrip
    assert actual_out.exists()
    assert actual_out.stat().st_size > 0
    roundtrip = raster_exposure.readbin(actual_out)
    assert np.array_equal(
        roundtrip["data"], np.unpackbits(ptrn, bitorder=raster_exposure.bitorder)
    )

    # Check Content Hash
    with open(actual_out, "rb") as f:
        file_hash = hashlib.md5(f.read()).hexdigest()

    if file_hash != expected_hash:
        print("\nHash Mismatch! Logic changed.")
        print(f"Expected: {expected_hash}")
        print(f"Got:      {file_hash}")
        print(
            "If the changes are intentional (e.g. bug fix), update the test variable."
        )

    assert file_hash == expected_hash


def test_roundtrip_integrity(raster_exposure, tmp_path):
    """
    Checks if Write -> Read produces the same parameters.
    This validates the readbin/writebin symmetry and the new IO module.
    """
    # 1. Mock Geometry
    # We can write directly to the params dict
    raster_exposure.params["samplexsize"] = 50.0  # mm
    raster_exposure.params["sampleysize"] = 50.0  # mm
    raster_exposure.params["facetsinlane"] = 100  # Aantal facetten per baan

    # Calculate expected sizes
    params = raster_exposure.params
    lanes = int(np.ceil(params["samplexsize"] / params["lanewidth"]))
    facets = int(params["facetsinlane"])
    bytes_line = int(np.ceil(params["bitsinscanline"] / 8))

    # 2. Create deterministic random data
    rng = np.random.default_rng(42)
    dummy_data = rng.integers(0, 255, size=lanes * facets * bytes_line, dtype=np.uint8)

    # 3. Write
    # The new writebin supports full paths, no need for .name hacks
    out_file = tmp_path / "test_roundtrip.bin"
    raster_exposure.writebin(dummy_data, out_file)

    # 4. Read back
    # The new readbin supports full paths too
    result = raster_exposure.readbin(out_file)
    r_meta = result["metadata"]
    r_bits = result["data"]

    # 5. Verify
    assert r_meta["facetsinlane"] == facets
    assert r_meta["lanes"] == lanes
    assert np.allclose(r_meta["lanewidth"], params["lanewidth"])

    r_bytes = np.packbits(r_bits, bitorder=raster_exposure.bitorder)
    assert len(r_bytes) == len(dummy_data)
    assert np.array_equal(r_bytes, dummy_data)


def test_rle_events_roundtrip_and_compression(raster_exposure, tmp_path):
    """
    Verifies lossless RLE event roundtrip (bits -> events -> bits) and
    confirming sparse PCB-like scanlines compress far below raw bitstream size.
    """
    laser_ticks = int(raster_exposure.params["laser_ticks"])
    scanline_length = int(raster_exposure.params["bitsinscanline"])

    rng = np.random.default_rng(123)
    dense_line = rng.integers(0, 2, size=scanline_length, dtype=np.uint8)
    sparse_line = np.zeros(scanline_length, dtype=np.uint8)
    sparse_line[100:125] = 1
    sparse_line[600:640] = 1

    for line in (np.zeros(scanline_length, dtype=np.uint8), sparse_line, dense_line):
        events = io.bits_to_events_np(line, laser_ticks)
        reconstructed_np = io.events_to_bits_np(events, scanline_length, laser_ticks)
        reconstructed_ulab = np.array(
            ulabext.events_to_bits(events, scanline_length, laser_ticks), dtype=np.uint8
        )
        assert np.array_equal(reconstructed_np, line)
        assert np.array_equal(reconstructed_ulab, line)

    # Verify sparse pattern file roundtrip & size reduction vs uncompressed raw bits
    raster_exposure.params["samplexsize"] = 20.0
    raster_exposure.params["sampleysize"] = 20.0
    raster_exposure.params["facetsinlane"] = 40
    params = raster_exposure.params
    lanes = int(np.ceil(params["samplexsize"] / params["lanewidth"]))
    facets = int(params["facetsinlane"])

    pattern_bits = np.zeros((lanes * facets, scanline_length), dtype=np.uint8)
    pattern_bits[:, 100:120] = 1
    packed = np.packbits(pattern_bits.flatten(), bitorder=raster_exposure.bitorder)

    out_file = tmp_path / "sparse_pcb.bin"
    raster_exposure.writebin(packed, out_file)
    result = raster_exposure.readbin(out_file)
    assert np.array_equal(result["data"], pattern_bits.flatten())
    assert raster_exposure._last_output_path.stat().st_size < packed.nbytes
