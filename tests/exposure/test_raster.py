import hashlib
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from hexastorm import ulabext
from hexastorm.exposure import (
    ExposureEngine,
    LaserheadMount,
    PrismCurve,
    RasterExposure,
    SubstratePlacement,
    VectorExposure,
    io,
)
from hexastorm.exposure.raster import quantize_grayscale


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
    # using tick-native 24 MHz step_half_period and 10 um samplegridsize software raster grid (560 samples).
    expected_hash = "d10bf66c9d33a47017b05f4d2cc7d62c"

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
    scanline_ticks = int(raster_exposure.params["scanline_ticks"])
    scanline_length = int(raster_exposure.params["bitsinscanline"])

    rng = np.random.default_rng(123)
    dense_line = rng.integers(0, 2, size=scanline_length, dtype=np.uint8)
    sparse_line = np.zeros(scanline_length, dtype=np.uint8)
    sparse_line[100:125] = 1
    sparse_line[400:440] = 1

    for line in (np.zeros(scanline_length, dtype=np.uint8), sparse_line, dense_line):
        events = io.bits_to_events_np(line, scanline_ticks)
        reconstructed_np = io.events_to_bits_np(events, scanline_length, scanline_ticks)
        reconstructed_ulab = np.array(
            ulabext.events_to_bits(events, scanline_length, scanline_ticks),
            dtype=np.uint8,
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


def test_grayscale_rle_and_roundtrip(tmp_path):
    """Verify opt-in grayscale RLE quantization, LUT remapping, .pat roundtrip, and default binary isolation."""
    # 1. RLE helper checks: flat runs emit minimal events and roundtrip without loss
    scanline_ticks = 42_000
    n_samples = 560
    sample_line = np.zeros(n_samples, dtype=np.uint8)
    sample_line[50:150] = 85
    sample_line[150:300] = 170
    sample_line[300:400] = 255
    events = io.intensities_to_events_np(sample_line, scanline_ticks)
    assert len(events) == 4
    assert np.array_equal(
        io.events_to_intensities_np(events, n_samples, scanline_ticks), sample_line
    )
    assert np.array_equal(
        np.array(
            ulabext.events_to_intensities(events, n_samples, scanline_ticks),
            dtype=np.uint8,
        ),
        sample_line,
    )

    # 2. Custom gamma / Beer-Lambert-style LUT
    gamma_lut = [round(255 * ((i / 255.0) ** 2.0)) for i in range(256)]
    quantized = quantize_grayscale(
        np.array([0, 128, 255], dtype=np.uint8), gray_levels=16, intensity_lut=gamma_lut
    )
    assert quantized[0] == 0 and quantized[-1] == 255
    assert 0 < quantized[1] < 128

    # 3. Gradient image through ExposureEngine (grayscale=True vs default grayscale=False)
    grad_row = np.linspace(255, 0, 120, dtype=np.uint8)
    grad_img = np.tile(grad_row, (60, 1))
    png_path = tmp_path / "gradient.png"
    Image.fromarray(grad_img, mode="L").save(png_path)

    gray_engine = ExposureEngine(grayscale=True, gray_levels=16)
    gray_pat = tmp_path / "gradient_gray.pat"
    gray_engine.generate(png_path, output_path=gray_pat, pixelsize=0.05)
    assert gray_engine._last_mode == "raster"

    raw_gray_events = gray_engine.readbin_events()["events"]
    intermediate_intensities = {
        inten for _d, evs in raw_gray_events for _t, inten in evs if 0 < inten < 255
    }
    assert len(intermediate_intensities) >= 4
    # RLE emits far fewer events per line than the full 560-sample grid
    assert max(len(evs) for _d, evs in raw_gray_events) < 200

    gray_res = gray_engine.readbin()
    assert np.any((gray_res["data"] > 0) & (gray_res["data"] < 255))
    plot_img = gray_engine.plot(gray_res, filename="test_grayscale_plot")
    assert plot_img.ndim == 3 and np.any(plot_img > 0)

    # Default ExposureEngine stays strictly binary (0 / 255 only)
    bin_engine = ExposureEngine()
    bin_pat = tmp_path / "gradient_bin.pat"
    bin_engine.generate(png_path, output_path=bin_pat, pixelsize=0.05)
    bin_events = bin_engine.readbin_events()["events"]
    bin_intensities = {inten for _d, evs in bin_events for _t, inten in evs}
    assert bin_intensities <= {0, 255}


def test_native_bitmap_not_resized_and_preserves_fine_features(tmp_path):
    """Verify native bitmaps are not resized onto samplegridsize and resolve 5 um stripes."""
    px_mm = 0.005  # 5 um/px (< 10 um samplegridsize fallback)
    arr = np.full((40, 400), 255, dtype=np.uint8)
    arr[8:32, [100, 102, 104]] = 0  # three 5 um black lines with 5 um gaps

    png_path = tmp_path / "fine_5um.png"
    Image.fromarray(arr, mode="L").save(png_path)

    ras = RasterExposure()
    assert ras.imgtopil(png_path).size == (400, 40)

    engine = ExposureEngine()
    pat_out = tmp_path / "fine_5um.pat"
    engine.generate(png_path, output_path=pat_out, pixelsize=px_mm)
    assert engine._last_mode == "raster"

    raw = engine.readbin_events()
    active_lines = [evs for _d, evs in raw["events"] if evs]
    assert len(active_lines) > 10

    mid_evs = active_lines[len(active_lines) // 2]
    assert len(mid_evs) == 6
    assert [v for _t, v in mid_evs] == [255, 0, 255, 0, 255, 0]
    ticks = [t for t, _v in mid_evs]
    assert ticks == sorted(ticks) and len(set(ticks)) == 6

    curve = PrismCurve(engine.params, invert_hw_scan_dir=True)
    u_shift = curve.raw_u_last + curve.xstart_val
    expected_ticks = [
        curve.u_to_tick(u - u_shift)
        for u in (0.500, 0.505, 0.510, 0.515, 0.520, 0.525)
    ]
    assert all(abs(t - exp) <= 1 for t, exp in zip(ticks, expected_ticks))

    # Same bitmap saved with 5080 DPI metadata (25.4 / 5080 = 0.005 mm/px), no explicit pixelsize
    dpi_png = tmp_path / "fine_5um_dpi.png"
    Image.fromarray(arr, mode="L").save(dpi_png, dpi=(5080, 5080))
    dpi_engine = ExposureEngine()
    dpi_engine.generate(dpi_png, output_path=tmp_path / "fine_5um_dpi.pat")
    dpi_raw = dpi_engine.readbin_events()
    assert dpi_raw["metadata"] == raw["metadata"]
    assert dpi_raw["events"] == raw["events"]


def test_native_bitmap_matches_vector_and_handles_transforms(tmp_path):
    """Verify native bitmap slicing matches VectorExposure across mounts/corrections and emits grayscale bands."""
    px_mm = 0.05
    arr = np.full((60, 120), 255, dtype=np.uint8)  # 6.0 x 3.0 mm
    arr[10:40, 20:50] = 0  # x in [1.0, 2.5] mm, y in [1.0, 2.5] mm
    png_path = tmp_path / "rect_50um.png"
    Image.fromarray(arr, mode="L").save(png_path)

    rect_poly = np.array(
        [[1.0, 1.0], [2.5, 1.0], [2.5, 2.5], [1.0, 2.5]], dtype=np.float64
    )
    correction = {
        "0": {"scan": 0.0, "orth": 0.0},
        "1": {"scan": 0.04, "orth": -0.05},
        "2": {"scan": -0.03, "orth": 0.04},
        "3": {"scan": 0.02, "orth": 0.0},
    }
    cases = [
        ("default", False, SubstratePlacement(align="laserhead"), LaserheadMount()),
        (
            "non_inv",
            False,
            SubstratePlacement(align="laserhead"),
            LaserheadMount(invert_hw_scan_dir=False),
        ),
        (
            "rot_align",
            False,
            SubstratePlacement(align="machine"),
            LaserheadMount(rotation_deg=90.0),
        ),
        ("cor_e1", correction, SubstratePlacement(align="laserhead"), LaserheadMount()),
    ]

    for tag, cor, placement, mount in cases:
        vec = VectorExposure(
            correction=cor, exposures=1, placement=placement, mount=mount
        )
        vec_out = tmp_path / f"{tag}_vec.pat"
        vec.slice_polygons([rect_poly], width_mm=6.0, height_mm=3.0, filepath=vec_out)
        vec_ev = vec.readbin_events(vec_out)

        eng = ExposureEngine(
            correction=cor, exposures=1, placement=placement, mount=mount
        )
        bmp_out = tmp_path / f"{tag}_bmp.pat"
        eng.generate(png_path, output_path=bmp_out, pixelsize=px_mm)
        bmp_ev = eng.readbin_events()

        assert bmp_ev["metadata"] == vec_ev["metadata"]
        vec_lines = vec_ev["events"]
        bmp_lines = bmp_ev["events"]
        assert len(vec_lines) == len(bmp_lines)

        vec_active = [evs for _d, evs in vec_lines if evs]
        bmp_active = [evs for _d, evs in bmp_lines if evs]
        assert abs(len(vec_active) - len(bmp_active)) <= 2

        matched_lines = 0
        for (d_v, ev_v), (d_b, ev_b) in zip(vec_lines, bmp_lines):
            assert d_v == d_b
            if len(ev_v) == 2 and len(ev_b) == 2:
                matched_lines += 1
                assert ev_v[0][1] == ev_b[0][1] == 255
                assert ev_v[1][1] == ev_b[1][1] == 0
                assert abs(ev_v[0][0] - ev_b[0][0]) <= 1
                assert abs(ev_v[1][0] - ev_b[1][0]) <= 1
        assert matched_lines > 100

    # Grayscale 3-band PNG at native 50 um resolution -> strict monotonic transition ticks
    arr_g = np.full((60, 120), 255, dtype=np.uint8)
    arr_g[10:40, 20:40] = 170  # dose ~85
    arr_g[10:40, 40:60] = 85  # dose ~170
    arr_g[10:40, 60:80] = 0  # dose 255
    gray_png = tmp_path / "bands_50um.png"
    Image.fromarray(arr_g, mode="L").save(gray_png)

    gray_eng = ExposureEngine(grayscale=True, gray_levels=32)
    gray_eng.generate(gray_png, output_path=tmp_path / "bands_50um.pat", pixelsize=px_mm)
    gray_active = [evs for _d, evs in gray_eng.readbin_events()["events"] if evs]
    mid_g = gray_active[len(gray_active) // 2]
    assert len(mid_g) == 4
    g_ticks = [t for t, _v in mid_g]
    assert g_ticks == sorted(g_ticks) and len(set(g_ticks)) == 4
    v0, v1, v2, v3 = [v for _t, v in mid_g]
    assert abs(v0 - 85) <= 10 and abs(v1 - 170) <= 10 and v2 == 255 and v3 == 0
