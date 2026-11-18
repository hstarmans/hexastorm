import struct
import zlib

import numpy as np

from hexastorm.interpolator.interpolator import Interpolator
from hexastorm.interpolator.vector import (
    LaserheadMount,
    VectorInterpolator,
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
    interp = VectorInterpolator(correction=False, exposures=1)
    curve = interp.curve

    u_grid = np.linspace(curve.u_base_min, curve.u_base_max, 500)
    ticks = curve.u_to_tick(u_grid)

    # Strict monotonicity across u -> tick across full extended pixel aperture [0, total_ticks]
    assert np.all(np.diff(ticks) >= 0)
    assert int(ticks[0]) == 0
    assert int(ticks[-1]) == curve.total_ticks

    # Lane nominal bounds [0, lanewidth] sit at first/last pixel centers (half a pixel inward)
    half_px_ticks = curve.laser_ticks // 2
    assert curve.u_to_tick(0.0) == half_px_ticks
    assert curve.u_to_tick(curve.lanewidth) == curve.total_ticks - half_px_ticks

    # Verify roundtrip tick -> u -> tick matches within <= 1 FPGA clock tick
    sample_ticks = np.linspace(0, curve.total_ticks, 200)
    u_recovered = curve.tick_to_u(sample_ticks)
    ticks_reencoded = curve.u_to_tick(u_recovered)
    assert np.max(np.abs(ticks_reencoded - np.rint(sample_ticks))) <= 1


def test_vector_vs_reference_interpolator_agreement(tmp_path):
    """Compare VectorInterpolator output against golden reference Interpolator on PCB SVG geometry."""
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

    ref_interp = Interpolator(correction=False, exposures=1)
    ref_ptrn = ref_interp.patternfile(svg_file)
    ref_bin = tmp_path / "ref_pcb.bin"
    ref_interp.writebin(ref_ptrn, ref_bin)
    ref_res = ref_interp.readbin(ref_interp._last_output_path)

    vec_interp = VectorInterpolator(correction=False, exposures=1)
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
    """Verify a 2 um edge shift (< ~4.1 um pixel pitch) yields a sub-pixel clock-tick shift."""
    vec_interp = VectorInterpolator(correction=False, exposures=1)
    bits_scanline = int(vec_interp.params["bitsinscanline"])
    laser_ticks = int(vec_interp.params["laser_ticks"])
    assert laser_ticks > 1

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

    # Shift right edge/left edge by +0.002 mm (2 um), smaller than one ~4.1 um laser pixel
    lines_a = slice_single_rect(1.000, "rect_a.bin")
    lines_b = slice_single_rect(1.002, "rect_b.bin")
    assert len(lines_a) == len(lines_b) and len(lines_a) > 10

    tick_on_a = lines_a[len(lines_a) // 2][0][0]
    tick_on_b = lines_b[len(lines_b) // 2][0][0]
    tick_diff = tick_on_b - tick_on_a

    # Both lie within ~1 pixel pitch (~60 ticks), yet distinctly resolve the 2 um shift (~25-35 ticks)
    assert 0 < tick_diff < laser_ticks
    assert 0 <= tick_on_a < tick_on_b < bits_scanline * laser_ticks


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

    vec_uncor = VectorInterpolator(correction=False, exposures=1)
    vec_cor = VectorInterpolator(correction=correction, exposures=2)

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
    """Verify VectorInterpolator.plotptrn() and CameraCalibrationGen(..., use_vector=True)."""
    from hexastorm.interpolator.patterns.camera import CameraCalibrationGen

    vec_interp = VectorInterpolator(correction=False, exposures=1)
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
        use_vector=True,
    )
    cam_pat.generate_vertical_jitter_test()
    res_jitter = cam_pat.interpolator.readbin()
    assert len(res_jitter["data"]) > 0
    assert np.any(res_jitter["data"] == 1)

    cam_pat.generate_dot_grid_test(num_dots_y=1)
    res_grid = cam_pat.interpolator.readbin()
    assert len(res_grid["data"]) > 0
    assert np.any(res_grid["data"] == 1)

    cam_pat.generate_blank_test()
    res_blank = cam_pat.interpolator.readbin()
    assert len(res_blank["data"]) > 0
    assert np.any(res_blank["data"] == 1)

    cam_plot = cam_pat.interpolator.plotptrn(res_grid, filename="test_cam_vector_grid")
    assert cam_plot.ndim == 3
    assert np.any(cam_plot > 0)
