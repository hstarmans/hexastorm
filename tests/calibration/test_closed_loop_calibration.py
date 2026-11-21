import numpy as np

from hexastorm.calibration import (
    FakeScanhead,
    PrismCurveLite,
    VirtualStackerUlab,
    analyze_facets_ulab,
    make_dot_centers_mm,
    make_tick_dot_events,
    slice_dot_grid_events,
)
from hexastorm.config import PlatformConfig
from hexastorm.exposure.patterns.camera import CameraCalibrationGen


def test_1d_baseline_calibration_recovers_ground_truth():
    """Verify 1D FakeScanhead + analyze_facets_ulab recovers offsets, tilt angle, and 1:3 eccentricity."""
    params = PlatformConfig(test=False).get_optical_params(
        correction=False, exposures=1
    )
    true_offsets = {
        0: (18.0, -12.0),
        1: (-52.0, 41.0),
        2: (29.0, -25.0),
        3: (-15.0, 14.0),
    }
    scanhead = FakeScanhead(
        params=params,
        facet_offsets_um=true_offsets,
        rotation_deg=2.1,
        pixel_size_um=4.4,
        spot_sigma_scan_um=10.0,
        spot_sigma_orth_um=28.0,
    )
    dark = scanhead.render_dark_frame()

    # Use 6 evenly spaced calibration dots across the sensor FOV (~1.5..4.1 mm)
    curve = PrismCurveLite(params, invert_hw_scan_dir=True)
    centers = make_dot_centers_mm(1.5, 4.1, num_dots=6, margin_mm=0.2)
    events = make_tick_dot_events(centers, dot_width_mm=0.02, curve=curve)
    frames = [scanhead.render_events(events, facet=f) for f in range(4)]

    res = analyze_facets_ulab(
        frames, dark_frame=dark, pixelsize=4.4, compute_rotation=True
    )

    ref_s = np.median([v[0] for v in true_offsets.values()])
    ref_o = np.median([v[1] for v in true_offsets.values()])

    for f in range(4):
        f_res = res[str(f)]
        exp_s = true_offsets[f][0] - ref_s
        exp_o = true_offsets[f][1] - ref_o
        assert abs(f_res["Scan_shift_um"] - exp_s) <= 3.0
        assert abs(f_res["Orth_shift_um"] - exp_o) <= 3.0
        assert abs(f_res["global_angle_deg"] - 2.1) <= 0.15
        assert f_res["mean_eccentricity"] > 0.7


def test_2d_closed_loop_vector_exposure_and_virtual_stacker(tmp_path):
    """Verify full closed-loop calibration with CameraCalibrationGen, VectorExposure, and VirtualStackerUlab."""
    params = PlatformConfig(test=False).get_optical_params(
        correction=False, exposures=1
    )
    true_offsets = {
        0: (14.0, -9.0),
        1: (-55.0, 42.0),
        2: (28.0, -24.0),
        3: (-12.0, 15.0),
    }
    scanhead = FakeScanhead(
        params=params,
        facet_offsets_um=true_offsets,
        rotation_deg=1.5,
        pixel_size_um=4.4,
    )
    dark = scanhead.render_dark_frame()

    # 1. Baseline 1D calibration
    curve = PrismCurveLite(params, invert_hw_scan_dir=True)
    centers = make_dot_centers_mm(1.4, 4.2, num_dots=6, margin_mm=0.2)
    base_events = make_tick_dot_events(centers, dot_width_mm=0.02, curve=curve)
    frames_1d = [scanhead.render_events(base_events, facet=f) for f in range(4)]
    cal_1d = analyze_facets_ulab(
        frames_1d, dark_frame=dark, pixelsize=4.4, compute_rotation=True
    )
    global_angle = cal_1d["0"]["global_angle_deg"]
    measured_cal = {
        str(f): {
            "scan": cal_1d[str(f)]["Scan_shift_um"] / 1000.0,
            "orth": cal_1d[str(f)]["Orth_shift_um"] / 1000.0,
        }
        for f in range(4)
    }

    steps_mm = float(scanhead.params["stepsperline"])
    cfg = PlatformConfig(test=False)
    orth_steps_mm = float(cfg.motor_cfg["steps_mm"][cfg.motor_cfg["orth2lsrline"]])
    assert steps_mm > 0

    def eval_vector_grid(correction, tag: str) -> float:
        gen = CameraCalibrationGen(
            lower_pixel=140,
            upper_pixel=420,
            line_thickness_mm=0.08,
            correction=correction,
            exposures=4,
            mode="auto",
        )
        # Keep CameraCalibrationGen using tmp_path so test leaves repo output clean
        gen.engine.cfg.paths["patterns"]  # ensure init
        gen.generate_dot_grid_test(num_dots_y=1)
        out_pat = tmp_path / f"grid_{tag}.pat"
        gen.engine.generate(
            gen.engine.cfg.paths["svgs"] / "cam_test_grid_80um.svg",
            output_path=out_pat,
        )
        lines_events = gen.engine.readbin_events(out_pat)["events"]
        total_rows = len(lines_events)
        stackers = [
            VirtualStackerUlab(
                expected_total_steps=total_rows,
                exposures=4,
                px_size_um=4.4,
                steps_mm=orth_steps_mm,
                rotation_angle_deg=global_angle,
            )
            for _ in range(4)
        ]
        for row_idx, (_dir, evs) in enumerate(lines_events):
            if not evs:
                continue
            facet = row_idx % 4
            frame = scanhead.render_events(evs, facet=facet, v_stage_mm=0.0)
            stackers[facet].add_exposure(frame, num_steps=row_idx, dark_frame=dark)

        res = analyze_facets_ulab(
            [s.get_result() for s in stackers],
            dark_frame=None,
            pixelsize=4.4,
            compute_rotation=False,
            rotation_angle_deg=global_angle,
            max_diameter_um=320.0,
        )
        return max(
            max(abs(res[str(f)]["Scan_shift_um"]), abs(res[str(f)]["Orth_shift_um"]))
            for f in range(4)
        )

    err_before = eval_vector_grid(False, "before")
    err_after = eval_vector_grid(measured_cal, "after")
    flipped_cal = {
        k: {"scan": -v["scan"], "orth": -v["orth"]} for k, v in measured_cal.items()
    }
    err_flipped = eval_vector_grid(flipped_cal, "flipped")

    assert err_before > 40.0
    assert err_after < 10.0
    assert err_flipped > err_before * 1.5


def test_2d_closed_loop_analytical_esp32_slicer():
    """Verify MicroPython-safe slice_dot_grid_events + VirtualStackerUlab closes calibration loop."""
    params = PlatformConfig(test=False).get_optical_params(
        correction=False, exposures=1
    )
    true_offsets = {
        0: (16.0, -10.0),
        1: (-48.0, 38.0),
        2: (24.0, -22.0),
        3: (-10.0, 14.0),
    }
    scanhead = FakeScanhead(
        params=params,
        facet_offsets_um=true_offsets,
        rotation_deg=1.8,
        pixel_size_um=4.4,
    )
    dark = scanhead.render_dark_frame()

    curve = PrismCurveLite(params, invert_hw_scan_dir=True)
    centers = make_dot_centers_mm(1.4, 4.2, num_dots=6, margin_mm=0.2)
    base_events = make_tick_dot_events(centers, dot_width_mm=0.02, curve=curve)
    frames_1d = [scanhead.render_events(base_events, facet=f) for f in range(4)]
    cal_1d = analyze_facets_ulab(
        frames_1d, dark_frame=dark, pixelsize=4.4, compute_rotation=True
    )
    global_angle = cal_1d["0"]["global_angle_deg"]

    def run_2d(facet_offsets_mm: dict[int, tuple[float, float]]) -> float:
        plan = slice_dot_grid_events(
            u_min_mm=1.4,
            u_max_mm=4.2,
            params=params,
            num_dots_x=6,
            num_dots_y=1,
            dot_diameter_mm=0.08,
            exposures=4,
            steps_mm=100.0,
            facet_offsets_mm=facet_offsets_mm,
        )
        stackers = [
            VirtualStackerUlab(
                expected_total_steps=plan["total_rows"],
                exposures=4,
                px_size_um=4.4,
                steps_mm=100.0,
                rotation_angle_deg=global_angle,
            )
            for _ in range(4)
        ]
        for row_idx, facet, events in plan["rows"]:
            frame = scanhead.render_events(events, facet=facet, v_stage_mm=0.0)
            stackers[facet].add_exposure(frame, num_steps=row_idx, dark_frame=dark)

        res = analyze_facets_ulab(
            [s.get_result() for s in stackers],
            dark_frame=None,
            pixelsize=4.4,
            compute_rotation=False,
            rotation_angle_deg=global_angle,
            max_diameter_um=320.0,
        )
        return max(
            max(abs(res[str(f)]["Scan_shift_um"]), abs(res[str(f)]["Orth_shift_um"]))
            for f in range(4)
        )

    err_before = run_2d({f: (0.0, 0.0) for f in range(4)})
    corr_offsets = {
        f: (
            cal_1d[str(f)]["Scan_shift_um"] / 1000.0,
            cal_1d[str(f)]["Orth_shift_um"] / 1000.0,
        )
        for f in range(4)
    }
    err_after = run_2d(corr_offsets)

    assert err_before > 40.0
    assert err_after < 10.0


def test_white_noise_averaging():
    """Verify multi-iteration averaging reduces sensor white-noise error below 3.5 um."""
    params = PlatformConfig(test=False).get_optical_params(
        correction=False, exposures=1
    )
    true_offsets = {
        0: (15.0, -8.0),
        1: (-45.0, 35.0),
        2: (25.0, -20.0),
        3: (-10.0, 12.0),
    }
    scanhead = FakeScanhead(
        params=params,
        facet_offsets_um=true_offsets,
        rotation_deg=1.5,
        pixel_size_um=4.4,
        noise_std=12.0,
        seed=123,
    )
    dark = scanhead.render_dark_frame()
    curve = PrismCurveLite(params, invert_hw_scan_dir=True)
    centers = make_dot_centers_mm(1.4, 4.2, num_dots=6, margin_mm=0.2)
    events = make_tick_dot_events(centers, dot_width_mm=0.02, curve=curve)

    scan_acc = {f: [] for f in range(4)}
    orth_acc = {f: [] for f in range(4)}
    for _ in range(4):
        frames = [scanhead.render_events(events, facet=f) for f in range(4)]
        res = analyze_facets_ulab(
            frames, dark_frame=dark, pixelsize=4.4, compute_rotation=True
        )
        for f in range(4):
            scan_acc[f].append(res[str(f)]["Scan_shift_um"])
            orth_acc[f].append(res[str(f)]["Orth_shift_um"])

    ref_s = np.median([v[0] for v in true_offsets.values()])
    ref_o = np.median([v[1] for v in true_offsets.values()])
    for f in range(4):
        assert abs(np.mean(scan_acc[f]) - (true_offsets[f][0] - ref_s)) <= 3.5
        assert abs(np.mean(orth_acc[f]) - (true_offsets[f][1] - ref_o)) <= 3.5
