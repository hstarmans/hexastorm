import math
import zlib

import numpy as np
import pytest

from hexastorm.calibration.core import slice_dot_grid_events
from hexastorm.calibration.vector_lite import (
    HAS_VECTOR_CORE,
    decode_spi_scanlines,
    slice_dot_grid_vector_core,
    slice_polygons_to_spi,
)
from hexastorm.config import PlatformConfig, Spi
from hexastorm.exposure.vector import VectorExposure, VectorShape


@pytest.mark.skipif(not HAS_VECTOR_CORE, reason="_vector_core C extension not built")
def test_vector_lite_matches_vector_exposure_bit_for_bit(tmp_path):
    """Verify slice_polygons_to_spi matches VectorExposure byte-for-byte."""
    cfg = PlatformConfig(test=False)
    corr = {
        "0": {"scan": 0.015, "orth": -0.010},
        "1": {"scan": -0.025, "orth": 0.018},
        "2": {"scan": 0.008, "orth": -0.004},
        "3": {"scan": -0.012, "orth": 0.022},
    }
    exposure = VectorExposure(correction=corr, exposures=2, intensity=210)

    # Outer rectangle with a hole (evenodd) + separate overlapping square + circle
    outer = np.array(
        [[1.0, 0.4], [4.5, 0.4], [4.5, 2.2], [1.0, 2.2]], dtype=np.float64
    )
    hole = np.array(
        [[2.0, 0.9], [3.5, 0.9], [3.5, 1.7], [2.0, 1.7]], dtype=np.float64
    )
    angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
    circle = np.column_stack((6.2 + 0.4 * np.cos(angles), 1.3 + 0.4 * np.sin(angles)))
    box = np.array([[5.0, 0.3], [6.6, 0.3], [6.6, 0.8], [5.0, 0.8]], dtype=np.float64)

    shapes = [
        VectorShape(contours=[outer, hole], fill_rule="evenodd"),
        VectorShape(contours=[circle], fill_rule="nonzero"),
        box,
    ]
    width_mm = 8.5
    height_mm = 2.8
    pat_path = tmp_path / "ref_vector.pat"
    exposure.slice_polygons(shapes, width_mm=width_mm, height_mm=height_mm, filepath=pat_path)

    # Read reference raw SPI payload (12-byte <fII header + concatenated lane SPI buffers)
    raw_pat = zlib.decompress(pat_path.read_bytes())
    ref_spi_body = raw_pat[12:]

    # Build the exact same Laserhead-frame shapes via VectorExposure's CoordinateTransform
    coord = exposure._last_coord
    assert coord is not None
    u_size_mm, v_size_mm = coord.u_size_mm, coord.v_size_mm
    lite_shapes = []
    for item in shapes:
        sh = item if isinstance(item, VectorShape) else VectorShape(contours=[item])
        lite_shapes.append(
            ([coord.substrate_to_laserhead(c) for c in sh.contours], sh.fill_rule)
        )

    params = exposure.params
    lanes = int(params["lanes"])
    facets_inlane = int(params["facetsinlane"])
    steps_per_line = float(params["stepsperline"])

    lite_spi_parts = []
    total_active = 0
    for lane_idx in range(lanes):
        raw_lane, lane_active = slice_polygons_to_spi(
            polygons=lite_shapes,
            params=params,
            u_size_mm=u_size_mm,
            v_size_mm=v_size_mm,
            steps_per_line=steps_per_line,
            lane_idx=lane_idx,
            facets_inlane=facets_inlane,
            invert_hw_scan_dir=True,
            positiveresist=bool(params["positiveresist"]),
            default_intensity=210,
            n_lut=1025,
            include_stage_drift=True,
        )
        lite_spi_parts.append(raw_lane)
        total_active += lane_active

    assert total_active == exposure._last_active_lines
    assert b"".join(lite_spi_parts) == ref_spi_body
    assert len(cfg.laser_timing) > 0


@pytest.mark.skipif(not HAS_VECTOR_CORE, reason="_vector_core C extension not built")
def test_decode_spi_scanlines_and_dot_grid_parity():
    """Verify SPI scanline decoding and slice_dot_grid_vector_core parity."""
    cfg = PlatformConfig(test=False)
    params = cfg.get_optical_params(correction=False, exposures=1)
    offsets = {
        0: (0.012, -0.008),
        1: (-0.018, 0.014),
        2: (0.005, 0.009),
        3: (-0.007, -0.015),
    }

    plan_ana = slice_dot_grid_events(
        u_min_mm=1.2,
        u_max_mm=4.4,
        params=params,
        num_dots_x=6,
        num_dots_y=1,
        dot_diameter_mm=0.08,
        exposures=2,
        steps_mm=100.0,
        facet_offsets_mm=offsets,
    )
    plan_vec = slice_dot_grid_vector_core(
        u_min_mm=1.2,
        u_max_mm=4.4,
        params=params,
        num_dots_x=6,
        num_dots_y=1,
        dot_diameter_mm=0.08,
        exposures=2,
        steps_mm=100.0,
        facet_offsets_mm=offsets,
        circle_segments=48,
    )

    assert plan_vec["total_rows"] == plan_ana["total_rows"]
    assert plan_vec["dot_centers"] == plan_ana["dot_centers"]
    assert abs(len(plan_vec["rows"]) - len(plan_ana["rows"])) <= 1

    # Compare active rows at identical row indices
    ana_by_row = {r: (f, ev) for r, f, ev in plan_ana["rows"]}
    vec_by_row = {r: (f, ev) for r, f, ev in plan_vec["rows"]}
    shared_rows = sorted(set(ana_by_row) & set(vec_by_row))
    assert len(shared_rows) >= len(plan_ana["rows"]) - 1

    for r in shared_rows:
        f_a, ev_a = ana_by_row[r]
        f_v, ev_v = vec_by_row[r]
        assert f_a == f_v
        assert len(ev_v) == len(ev_a)
        for (tick_a, int_a), (tick_v, int_v) in zip(ev_a, ev_v):
            assert int_a == int_v
            assert abs(tick_a - tick_v) <= 8

    # Also validate decode_spi_scanlines with skip_empty=False on minimal synthetic buffer
    raw_spi, active = slice_polygons_to_spi(
        polygons=[[(2.0, 0.1), (2.2, 0.1), (2.2, 0.2), (2.0, 0.2)]],
        params=params,
        u_size_mm=float(params["lanewidth"]),
        v_size_mm=0.5,
        steps_per_line=0.5,
        facets_inlane=10,
        y_shift_override=0.05,
        include_stage_drift=False,
    )
    assert active > 0
    assert len(raw_spi) % (Spi.command_bytes + Spi.word_bytes) == 0
    all_rows = decode_spi_scanlines(raw_spi, num_facets=4, skip_empty=False)
    assert len(all_rows) == 10
