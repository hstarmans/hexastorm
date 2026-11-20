import struct
import zlib

import numpy as np
import pytest
from PIL import Image, ImageDraw

from hexastorm.exposure import (
    ExposureEngine,
    LaserheadMount,
    RasterExposure,
    VectorExposure,
    VectorShape,
)


def _extract_raw_events(bin_path):
    """Decode raw (tick, intensity) event tuples per scanline from a compressed .bin/.pat file."""
    with open(bin_path, "rb") as f:
        data = zlib.decompress(f.read())
    lanewidth, facets_in_lane, lanes = struct.unpack("<fII", data[:12])
    payload = data[12:]
    offset = 0
    lines_events = []
    for _ in range(lanes * facets_in_lane):
        word0 = payload[offset + 1 : offset + 9][::-1]
        offset += 9
        payload_int = int.from_bytes(word0[1:8], "little")
        direction = payload_int & 1
        event_words = (payload_int >> 1) & 0xFFFF
        events = []
        for _ in range(event_words):
            ev_bytes = payload[offset + 1 : offset + 9][::-1]
            offset += 9
            ev0, ev1 = struct.unpack("<II", ev_bytes)
            t0 = (ev0 >> 8) & 0xFFFFFF
            if t0 < 0xFFFFFF:
                events.append((t0, ev0 & 0xFF))
            t1 = (ev1 >> 8) & 0xFFFFFF
            if t1 < 0xFFFFFF:
                events.append((t1, ev1 & 0xFF))
        lines_events.append((direction, events))
    return lanewidth, facets_in_lane, lanes, lines_events


def test_prism_curve_inverse_roundtrip():
    """Verify PrismCurve forward/inverse mapping roundtrips within <= 1 FPGA clock tick."""
    interp = VectorExposure(correction=False, exposures=1)
    curve = interp.curve

    u_grid = np.linspace(curve.u_base_min, curve.u_base_max, 500)
    ticks = curve.u_to_tick(u_grid)

    # Strict monotonicity across u -> tick across full extended pixel aperture [0, total_ticks]
    assert np.all(np.diff(ticks) >= 0)
    assert int(ticks[0]) == 0
    assert int(ticks[-1]) == curve.total_ticks

    # Lane bounds [0, lanewidth] map directly onto full FPGA clock span [0, total_ticks]
    assert curve.u_to_tick(0.0) == 0
    assert curve.u_to_tick(curve.lanewidth) == curve.total_ticks

    # Verify roundtrip tick -> u -> tick matches within <= 1 FPGA clock tick
    sample_ticks = np.linspace(0, curve.total_ticks, 200)
    u_recovered = curve.tick_to_u(sample_ticks)
    ticks_reencoded = curve.u_to_tick(u_recovered)
    assert np.max(np.abs(ticks_reencoded - np.rint(sample_ticks))) <= 1


def test_vector_vs_reference_interpolator_agreement(tmp_path):
    """Compare VectorExposure output against golden reference RasterExposure on PCB SVG geometry."""
    svg_content = """<?xml version="1.0" encoding="utf-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="8.0mm" height="8.0mm" viewBox="0 0 8.0 8.0">
  <rect x="0" y="0" width="8.0" height="8.0" fill="#ffffff"/>
  <!-- Vertical copper track spanning lanes -->
  <rect x="0.8" y="1.0" width="0.5" height="6.0" fill="#000000"/>
  <!-- Horizontal track spanning across lane 0 -> lane 1 boundary (~2.86 mm) -->
  <rect x="0.5" y="3.5" width="5.5" height="0.6" fill="#000000"/>
  <!-- Stroked diagonal track overlapping a filled pad (tests union, no false hole) -->
  <line x1="1.0" y1="6.2" x2="6.0" y2="5.0" stroke="#000000" stroke-width="0.4" stroke-linecap="round"/>
  <circle cx="5.2" cy="2.0" r="1.1" fill="#000000"/>
  <!-- Ring pad with interior hole via evenodd -->
  <path d="M 3.0 1.2 L 4.4 1.2 L 4.4 2.6 L 3.0 2.6 Z M 3.4 1.6 L 4.0 1.6 L 4.0 2.2 L 3.4 2.2 Z"
        fill="#000000" fill-rule="evenodd"/>
</svg>
"""
    svg_file = tmp_path / "pcb_test.svg"
    svg_file.write_text(svg_content, encoding="utf-8")

    ref_interp = RasterExposure(correction=False, exposures=1)
    ref_ptrn = ref_interp.patternfile(svg_file)
    ref_bin = tmp_path / "ref_pcb.bin"
    ref_interp.writebin(ref_ptrn, ref_bin)
    ref_res = ref_interp.readbin(ref_interp._last_output_path)

    vec_interp = VectorExposure(correction=False, exposures=1)
    vec_bin = tmp_path / "vec_pcb.bin"
    vec_interp.svg_to_bin(svg_file, vec_bin)
    vec_res = vec_interp.readbin(vec_bin)

    assert vec_res["metadata"]["lanes"] == ref_res["metadata"]["lanes"]
    assert vec_res["metadata"]["facetsinlane"] == ref_res["metadata"]["facetsinlane"]
    assert np.isclose(
        vec_res["metadata"]["lanewidth"],
        ref_res["metadata"]["lanewidth"],
        atol=1e-4,
    )

    ref_bits = ref_res["data"]
    vec_bits = vec_res["data"]
    assert ref_bits.shape == vec_bits.shape
    agreement = float(np.mean(ref_bits == vec_bits))
    assert agreement > 0.985, f"Vector vs reference agreement too low: {agreement:.4f}"


def test_vector_subpixel_tick_precision(tmp_path):
    """Verify a 2 um edge shift (< ~2.86 um software grid pitch) yields a sub-grid clock-tick shift."""
    vec_interp = VectorExposure(correction=False, exposures=1)
    scanline_ticks = int(vec_interp.params["scanline_ticks"])
    bits_scanline = int(vec_interp.params["bitsinscanline"])
    raster_tick_step = scanline_ticks // bits_scanline
    assert raster_tick_step > 1

    def slice_single_rect(x0_mm: float, name: str):
        poly = np.array(
            [
                [x0_mm, 0.2],
                [x0_mm + 0.5, 0.2],
                [x0_mm + 0.5, 1.0],
                [x0_mm, 1.0],
            ],
            dtype=np.float64,
        )
        out = tmp_path / name
        vec_interp.slice_polygons([poly], width_mm=2.5, height_mm=1.2, filepath=out)
        _lw, _fl, _ln, lines = _extract_raw_events(out)
        active = [evs for _d, evs in lines if evs]
        return active

    # Shift left edge by +0.002 mm (2 um), smaller than one ~2.86 um software raster step
    lines_a = slice_single_rect(1.000, "rect_a.bin")
    lines_b = slice_single_rect(1.002, "rect_b.bin")
    assert len(lines_a) == len(lines_b) and len(lines_a) > 10

    tick_on_a = lines_a[len(lines_a) // 2][0][0]
    tick_on_b = lines_b[len(lines_b) // 2][0][0]
    tick_diff = tick_on_b - tick_on_a

    # Both lie within 1 software raster step (~42 ticks), yet cleanly resolve the 2 um shift
    assert 0 < tick_diff < raster_tick_step
    assert 0 <= tick_on_a < tick_on_b < scanline_ticks


def test_vector_optical_correction_and_exposures(tmp_path):
    """Verify per-facet optical correction shifts ticks and exposures=2 doubles facetsinlane."""
    correction = {
        "0": {"scan": 0.0, "orth": 0.0},
        "1": {"scan": 0.050, "orth": 0.100},
        "2": {"scan": -0.040, "orth": -0.050},
        "3": {"scan": 0.020, "orth": 0.0},
    }
    poly = VectorShape(
        contours=[
            np.array([[0.5, 0.2], [2.0, 0.2], [2.0, 2.0], [0.5, 2.0]], dtype=np.float64)
        ],
        fill_rule="evenodd",
    )

    vec_uncor = VectorExposure(correction=False, exposures=1)
    vec_cor = VectorExposure(correction=correction, exposures=2)

    out_uncor = tmp_path / "uncor.bin"
    out_cor = tmp_path / "cor.bin"
    vec_uncor.slice_polygons([poly], width_mm=2.5, height_mm=2.2, filepath=out_uncor)
    vec_cor.slice_polygons([poly], width_mm=2.5, height_mm=2.2, filepath=out_cor)

    res_uncor = vec_uncor.readbin(out_uncor)
    res_cor = vec_cor.readbin(out_cor)
    assert res_cor["metadata"]["lanes"] == res_uncor["metadata"]["lanes"] == 1
    assert (
        res_cor["metadata"]["facetsinlane"]
        >= res_uncor["metadata"]["facetsinlane"] * 2 - 1
    )

    # Confirm facet 1 scan shift moves facet 1 events relative to facet 0
    _lw, _fl, _ln, events_cor = _extract_raw_events(out_cor)
    mid = len(events_cor) // 2
    mid -= mid % 4  # align to facet 0
    tick0 = events_cor[mid][1][0][0]
    tick1 = events_cor[mid + 1][1][0][0]
    assert tick0 != tick1


def test_laserhead_mount_orientations():
    """Verify LaserheadMount rotation and mirroring transform polygon boxes and vertices."""
    pts = np.array([[1.0, 2.0], [4.0, 2.0], [4.0, 5.0]], dtype=np.float64)

    mount_rot90 = LaserheadMount(rotation_deg=90.0)
    t_pts, u_size, v_size = mount_rot90.transform_points(
        pts, width_mm=10.0, height_mm=6.0
    )
    assert np.allclose([u_size, v_size], [6.0, 10.0], atol=1e-9)
    # CCW 90 deg: (x, y) -> (-y, x), shifted by +height_mm in u: (6 - y, x)
    expected_rot90 = np.array([[4.0, 1.0], [4.0, 4.0], [1.0, 4.0]], dtype=np.float64)
    assert np.allclose(t_pts, expected_rot90, atol=1e-9)

    mount_mir_u = LaserheadMount(rotation_deg=0.0, mirror_u=True)
    m_pts, mu_size, mv_size = mount_mir_u.transform_points(
        pts, width_mm=10.0, height_mm=6.0
    )
    assert np.allclose([mu_size, mv_size], [10.0, 6.0], atol=1e-9)
    expected_mir_u = np.array([[9.0, 2.0], [6.0, 2.0], [6.0, 5.0]], dtype=np.float64)
    assert np.allclose(m_pts, expected_mir_u, atol=1e-9)


def test_vector_plotptrn_and_camera_patterns(tmp_path):
    """Verify VectorExposure.plotptrn() and CameraCalibrationGen(..., mode='auto')."""
    from hexastorm.exposure.patterns.camera import CameraCalibrationGen

    vec_interp = VectorExposure(correction=False, exposures=1)
    poly = np.array([[0.5, 0.2], [2.0, 0.2], [2.0, 1.5], [0.5, 1.5]], dtype=np.float64)
    out_bin = tmp_path / "vector_plot_test.bin"
    vec_interp.slice_polygons([poly], width_mm=2.5, height_mm=1.8, filepath=out_bin)

    plot_img = vec_interp.plotptrn(filename="test_vector_plot")
    assert plot_img is not None
    assert plot_img.ndim == 3
    assert np.any(plot_img > 0)
    assert (vec_interp.cfg.paths["debug"] / "test_vector_plot.png").exists()

    cam_pat = CameraCalibrationGen(
        lower_pixel=200,
        upper_pixel=500,
        line_thickness_mm=0.15,
        mode="auto",
    )
    cam_pat.generate_vertical_jitter_test()
    assert cam_pat.engine._last_mode == "vector"
    res_jitter = cam_pat.engine.readbin()
    assert len(res_jitter["data"]) > 0
    assert np.any(res_jitter["data"] == 1)

    cam_pat.generate_dot_grid_test(num_dots_y=1)
    res_grid = cam_pat.engine.readbin()
    assert len(res_grid["data"]) > 0
    assert np.any(res_grid["data"] == 1)

    cam_pat.generate_blank_test()
    res_blank = cam_pat.engine.readbin()
    assert len(res_blank["data"]) > 0
    assert np.any(res_blank["data"] == 1)

    cam_plot = cam_pat.engine.plot(res_grid, filename="test_cam_vector_grid")
    assert cam_plot.ndim == 3
    assert np.any(cam_plot > 0)


def test_exposure_engine_dispatch_svg_and_raster(tmp_path):
    """Verify ExposureEngine dispatches SVGs to vector, images to raster, and validates modes."""
    svg_file = tmp_path / "sample.svg"
    svg_file.write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="5.0mm" height="5.0mm" viewBox="0 0 5.0 5.0">
  <rect x="0" y="0" width="5.0" height="5.0" fill="#ffffff"/>
  <rect x="1.0" y="1.0" width="2.0" height="2.5" fill="#000000"/>
</svg>
""",
        encoding="utf-8",
    )

    png_file = tmp_path / "sample.png"
    img = Image.new("RGB", (120, 120), "white")
    ImageDraw.Draw(img).rectangle([20, 20, 90, 90], fill="black")
    img.save(png_file)

    engine = ExposureEngine(mode="auto")

    svg_out = tmp_path / "out_svg.pat"
    engine.generate(svg_file, output_path=svg_out)
    assert engine._last_mode == "vector"
    svg_res = engine.readbin()
    assert np.any(svg_res["data"] == 1)
    assert np.any(engine.plot(svg_res, filename="test_dispatch_svg") > 0)

    png_out = tmp_path / "out_png.pat"
    engine.generate(png_file, output_path=png_out, pixelsize=0.05)
    assert engine._last_mode == "raster"
    png_res = engine.readbin()
    assert np.any(png_res["data"] == 1)
    assert np.any(engine.plot(png_res, filename="test_dispatch_png") > 0)

    svg_raster_out = tmp_path / "out_svg_raster.pat"
    engine.generate(svg_file, output_path=svg_raster_out, mode="raster")
    assert engine._last_mode == "raster"
    assert np.any(engine.readbin()["data"] == 1)

    with pytest.raises(ValueError, match="requires an .svg input"):
        engine.generate(png_file, mode="vector")


def test_machine_pattern_vector_vs_raster():
    """Verify LaserCalibrationGen supports mode='auto' (vector) and mode='raster' on combined grid."""
    from hexastorm.exposure.patterns.machine import LaserCalibrationGen

    gen_vec = LaserCalibrationGen(mode="auto")
    gen_vec.generate_combined_test()
    assert gen_vec.engine._last_mode == "vector"
    res_vec = gen_vec.engine.readbin()

    gen_ras = LaserCalibrationGen(mode="raster")
    gen_ras.generate_combined_test()
    assert gen_ras.engine._last_mode == "raster"
    res_ras = gen_ras.engine.readbin()

    assert res_vec["metadata"]["lanes"] == res_ras["metadata"]["lanes"]
    assert res_vec["metadata"]["facetsinlane"] == res_ras["metadata"]["facetsinlane"]
    assert np.isclose(
        res_vec["metadata"]["lanewidth"], res_ras["metadata"]["lanewidth"], atol=1e-4
    )
    assert np.any(res_vec["data"] == 1)
    assert res_vec["data"].shape == res_ras["data"].shape
    agreement = float(np.mean(res_vec["data"] == res_ras["data"]))
    assert agreement > 0.97, (
        f"Machine combined grid vector/raster agreement {agreement:.4f} too low"
    )


def test_vector_c_core_matches_python_fallback(tmp_path):
    """Verify the compiled C99 core matches the pure-Python fallback bit-for-bit."""
    from hexastorm.exposure import vector as vector_mod
    from hexastorm.exposure.vector import parse_svg_polygons

    assert vector_mod._HAS_C_CORE is True

    svg_content = """<?xml version="1.0" encoding="utf-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="9.0mm" height="7.5mm" viewBox="0 0 9.0 7.5">
  <rect x="0" y="0" width="9.0" height="7.5" fill="#ffffff"/>
  <rect x="0.6" y="0.8" width="0.5" height="5.8" fill="#000000"/>
  <rect x="0.3" y="3.2" width="7.2" height="0.55" fill="#000000"/>
  <line x1="1.2" y1="6.0" x2="7.5" y2="4.6" stroke="#000000" stroke-width="0.35" stroke-linecap="round"/>
  <circle cx="6.2" cy="1.8" r="1.0" fill="#000000"/>
  <path d="M 3.0 1.0 L 4.4 1.0 L 4.4 2.4 L 3.0 2.4 Z M 3.4 1.4 L 4.0 1.4 L 4.0 2.0 L 3.4 2.0 Z"
        fill="#000000" fill-rule="evenodd"/>
  <polygon points="1.8,4.8 2.6,4.2 3.1,5.2 2.2,5.6" fill="#000000" fill-rule="nonzero"/>
</svg>
"""
    svg_file = tmp_path / "parity.svg"
    svg_file.write_text(svg_content, encoding="utf-8")
    shapes, w_mm, h_mm = parse_svg_polygons(svg_file)

    correction = {
        "0": {"scan": 0.0, "orth": 0.0},
        "1": {"scan": 0.045, "orth": -0.060},
        "2": {"scan": -0.030, "orth": 0.080},
        "3": {"scan": 0.015, "orth": -0.020},
    }

    cases = [
        ("uncor", False, 1, False, LaserheadMount()),
        ("cor_e2", correction, 2, False, LaserheadMount()),
        ("pos_resist", False, 1, True, LaserheadMount(rotation_deg=90.0)),
        ("non_inv", False, 1, False, LaserheadMount(invert_hw_scan_dir=False)),
    ]

    for tag, cor, exps, pos_res, mount in cases:
        vec_c = VectorExposure(correction=cor, exposures=exps, mount=mount)
        vec_py = VectorExposure(correction=cor, exposures=exps, mount=mount)
        if pos_res:
            vec_c.params["positiveresist"] = 1.0
            vec_py.params["positiveresist"] = 1.0

        out_c = tmp_path / f"{tag}_c.pat"
        out_py = tmp_path / f"{tag}_py.pat"

        vec_c.slice_polygons(shapes, w_mm, h_mm, filepath=out_c, use_c=True)
        vec_py.slice_polygons(shapes, w_mm, h_mm, filepath=out_py, use_c=False)

        assert vec_c._last_active_lines == vec_py._last_active_lines
        assert zlib.decompress(out_c.read_bytes()) == zlib.decompress(
            out_py.read_bytes()
        ), f"Mismatch in {tag} decompressed SPI stream"
