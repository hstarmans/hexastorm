import numpy as np
import pytest
from PIL import Image, ImageDraw

from hexastorm.config import PlatformConfig
from hexastorm.exposure import (
    CoordinateTransform,
    ExposureEngine,
    LaserheadMount,
    MachineFrame,
    RasterExposure,
    SubstratePlacement,
    VectorExposure,
    plot_pattern,
)


def _asymmetric_svg(width_mm: float = 10.0, height_mm: float = 5.0) -> str:
    """Return an asymmetric L-shaped PCB-like SVG without any rotational symmetry."""
    return f"""<?xml version="1.0" encoding="utf-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width_mm}mm" height="{height_mm}mm" viewBox="0 0 {width_mm} {height_mm}">
  <rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="#ffffff"/>
  <rect x="0.8" y="0.7" width="1.1" height="3.6" fill="#000000"/>
  <rect x="0.8" y="3.2" width="7.8" height="1.1" fill="#000000"/>
  <circle cx="7.4" cy="1.6" r="0.65" fill="#000000"/>
</svg>
"""


def test_machine_frame_from_config():
    """Verify MachineFrame reflects orth2lsrline='x' CNC bed geometry."""
    cfg = PlatformConfig(test=False)
    mach = MachineFrame.from_config(cfg)
    assert mach.step_axis == "x"
    assert mach.lane_axis == "y"
    assert mach.head_rotation_deg == 90.0
    assert np.isclose(mach.x_size_mm, 200.0)
    assert np.isclose(mach.y_size_mm, 200.0)

    mach_y = MachineFrame(step_axis="y", x_size_mm=150.0, y_size_mm=120.0)
    assert mach_y.lane_axis == "x"
    assert mach_y.head_rotation_deg == 0.0


def test_default_laserhead_alignment_identity_and_machine_rotation():
    """Default align='laserhead' keeps Substrate==Laserhead and det=+1 Substrate->Machine."""
    mach = MachineFrame(step_axis="x")
    coord = CoordinateTransform(width_mm=10.0, height_mm=5.0, machine=mach)

    assert coord.is_identity_lh_to_sub is True
    assert np.allclose([coord.u_size_mm, coord.v_size_mm], [10.0, 5.0], atol=1e-9)
    assert np.allclose(
        [coord.machine_width_mm, coord.machine_height_mm], [5.0, 10.0], atol=1e-9
    )
    assert np.isclose(np.linalg.det(coord.sub_to_mach_matrix[:2, :2]), 1.0, atol=1e-9)
    assert np.isclose(np.linalg.det(coord.mach_to_lh_matrix[:2, :2]), 1.0, atol=1e-9)

    pts = np.array([[1.5, 2.0], [8.0, 4.0], [0.0, 0.0]], dtype=np.float64)
    mach_pts = coord.substrate_to_machine(pts)
    expected_mach = np.column_stack((pts[:, 1], 10.0 - pts[:, 0]))
    assert np.allclose(mach_pts, expected_mach, atol=1e-9)
    assert np.allclose(coord.substrate_to_laserhead(pts), pts, atol=1e-9)


def test_machine_alignment_pcb_dimensions_and_lane_count(tmp_path):
    """Align='machine' maps 10x5 PCB along X-gantry, reducing lanes vs 'laserhead'."""
    svg_path = tmp_path / "pcb_10x5.svg"
    svg_path.write_text(_asymmetric_svg(10.0, 5.0), encoding="utf-8")

    mach = MachineFrame(step_axis="x")
    coord_mach = CoordinateTransform(
        width_mm=10.0,
        height_mm=5.0,
        machine=mach,
        placement=SubstratePlacement(align="machine"),
    )
    assert coord_mach.is_identity_lh_to_sub is False
    assert np.allclose(
        [coord_mach.machine_width_mm, coord_mach.machine_height_mm],
        [10.0, 5.0],
        atol=1e-9,
    )
    assert np.allclose(
        [coord_mach.u_size_mm, coord_mach.v_size_mm], [5.0, 10.0], atol=1e-9
    )
    assert np.isclose(
        np.linalg.det(coord_mach.sub_to_lh_matrix[:2, :2]), 1.0, atol=1e-9
    )

    eng_default = ExposureEngine(
        mode="vector", placement=SubstratePlacement(align="laserhead")
    )
    eng_default.generate(svg_path, output_path=tmp_path / "default.pat")
    lanes_default = int(eng_default.params["lanes"])

    eng_machine = ExposureEngine(
        mode="vector", placement=SubstratePlacement(align="machine")
    )
    eng_machine.generate(svg_path, output_path=tmp_path / "machine.pat")
    lanes_machine = int(eng_machine.params["lanes"])

    assert lanes_machine < lanes_default


@pytest.mark.parametrize("align", ["laserhead", "machine"])
@pytest.mark.parametrize("rot_deg", [0.0, 90.0, 180.0, 270.0, 37.0])
@pytest.mark.parametrize(
    "mirrors", [(False, False, False, False), (True, False, False, True)]
)
def test_three_frame_roundtrips(align, rot_deg, mirrors):
    """Verify exact invertibility across Substrate <-> Machine <-> Laserhead."""
    mirror_x, mirror_y, mirror_u, mirror_v = mirrors
    mach = MachineFrame(step_axis="x", x_size_mm=200.0, y_size_mm=200.0)
    placement = SubstratePlacement(
        align=align,
        rotation_deg=rot_deg,
        mirror_x=mirror_x,
        mirror_y=mirror_y,
        origin_mm=(12.5, 25.0),
    )
    mount = LaserheadMount(rotation_deg=15.0, mirror_u=mirror_u, mirror_v=mirror_v)
    coord = CoordinateTransform(
        width_mm=10.0,
        height_mm=5.0,
        machine=mach,
        placement=placement,
        mount=mount,
    )

    pts = np.array(
        [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0], [3.2, 1.7]],
        dtype=np.float64,
    )
    mach_local = coord.substrate_to_machine(pts, include_origin=False)
    mach_abs = coord.substrate_to_machine(pts, include_origin=True)
    lh_pts = coord.substrate_to_laserhead(pts)

    assert np.allclose(coord.machine_to_substrate(mach_local), pts, atol=1e-9)
    assert np.allclose(
        coord.machine_to_substrate(mach_abs, include_origin=True), pts, atol=1e-9
    )
    assert np.allclose(coord.machine_to_laserhead(mach_local), lh_pts, atol=1e-9)
    assert np.allclose(coord.laserhead_to_machine(lh_pts), mach_local, atol=1e-9)
    assert np.allclose(coord.laserhead_to_substrate(lh_pts), pts, atol=1e-9)

    ux, vx = coord.laserhead_to_substrate_xy(lh_pts[:, 0], lh_pts[:, 1])
    assert np.allclose(np.column_stack((ux, vx)), pts, atol=1e-9)


def test_validate_fits_machine_origin_bounds():
    """Origin offsets pushing the substrate past machine travel limits must raise."""
    coord_ok = CoordinateTransform(
        width_mm=10.0,
        height_mm=5.0,
        machine=MachineFrame(step_axis="x", x_size_mm=50.0, y_size_mm=50.0),
        placement=SubstratePlacement(align="machine", origin_mm=(40.0, 45.0)),
    )
    coord_ok.validate_fits_machine()

    coord_oob = CoordinateTransform(
        width_mm=10.0,
        height_mm=5.0,
        machine=MachineFrame(step_axis="x", x_size_mm=50.0, y_size_mm=50.0),
        placement=SubstratePlacement(align="machine", origin_mm=(41.0, 45.0)),
    )
    with pytest.raises(ValueError, match="does not fit"):
        coord_oob.validate_fits_machine()


@pytest.mark.parametrize(
    "label,placement,mount",
    [
        ("default_lh", SubstratePlacement(align="laserhead"), LaserheadMount()),
        ("machine_align", SubstratePlacement(align="machine"), LaserheadMount()),
        (
            "mirror_x",
            SubstratePlacement(align="laserhead", mirror_x=True),
            LaserheadMount(),
        ),
        (
            "mount_rot90",
            SubstratePlacement(align="laserhead"),
            LaserheadMount(rotation_deg=90.0),
        ),
        (
            "mount_mirror_u",
            SubstratePlacement(align="laserhead"),
            LaserheadMount(mirror_u=True),
        ),
    ],
)
def test_vector_vs_raster_agreement_across_frames(tmp_path, label, placement, mount):
    """Verify VectorExposure and RasterExposure agree (>98.5%) on all placements/mounts."""
    svg_path = tmp_path / f"{label}.svg"
    svg_path.write_text(_asymmetric_svg(10.0, 5.0), encoding="utf-8")

    vec = VectorExposure(
        correction=False, exposures=1, placement=placement, mount=mount
    )
    vec_out = tmp_path / f"{label}_vec.pat"
    vec.svg_to_bin(svg_path, vec_out)
    vec_res = vec.readbin(vec_out)

    ras = RasterExposure(
        correction=False, exposures=1, placement=placement, mount=mount
    )
    ras_ptrn = ras.patternfile(svg_path)
    ras_out = tmp_path / f"{label}_ras.pat"
    ras.writebin(ras_ptrn, ras_out)
    ras_res = ras.readbin(ras._last_output_path)

    assert vec_res["metadata"]["lanes"] == ras_res["metadata"]["lanes"]
    assert vec_res["metadata"]["facetsinlane"] == ras_res["metadata"]["facetsinlane"]
    assert vec_res["data"].shape == ras_res["data"].shape

    agreement = float(np.mean(vec_res["data"] == ras_res["data"]))
    assert agreement > 0.985, (
        f"Vector vs Raster agreement too low for {label}: {agreement:.4f}"
    )


def test_raster_svg_vs_png_orientation_parity(tmp_path):
    """Verify RasterExposure gives the same orientation for SVG and PNG inputs."""
    w_mm, h_mm = 10.0, 5.0
    svg_path = tmp_path / "lshape.svg"
    svg_path.write_text(_asymmetric_svg(w_mm, h_mm), encoding="utf-8")

    # Render matching unrotated PNG at 0.05 mm/px (200x100 px)
    px_mm = 0.05
    w_px, h_px = round(w_mm / px_mm), round(h_mm / px_mm)
    img = Image.new("RGB", (w_px, h_px), "white")
    draw = ImageDraw.Draw(img)
    # Match SVG rects: (0.8..1.9, 0.7..4.3) and (0.8..8.6, 3.2..4.3)
    draw.rectangle(
        [
            round(0.8 / px_mm),
            round(0.7 / px_mm),
            round(1.9 / px_mm) - 1,
            round(4.3 / px_mm) - 1,
        ],
        fill="black",
    )
    draw.rectangle(
        [
            round(0.8 / px_mm),
            round(3.2 / px_mm),
            round(8.6 / px_mm) - 1,
            round(4.3 / px_mm) - 1,
        ],
        fill="black",
    )
    # Match circle cx=7.4, cy=1.6, r=0.65
    cx, cy, r = 7.4 / px_mm, 1.6 / px_mm, 0.65 / px_mm
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill="black")

    png_path = tmp_path / "lshape.png"
    img.save(png_path)

    ras_svg = RasterExposure(correction=False, exposures=1)
    bits_svg = np.unpackbits(ras_svg.patternfile(svg_path), bitorder="big")

    ras_png = RasterExposure(correction=False, exposures=1)
    bits_png = np.unpackbits(
        ras_png.patternfile(png_path, pixelsize=px_mm), bitorder="big"
    )

    assert bits_svg.shape == bits_png.shape
    parity = float(np.mean(bits_svg == bits_png))
    assert parity > 0.985, f"SVG vs PNG orientation mismatch: parity={parity:.4f}"


def test_plotter_views_and_aspect_ratio(tmp_path):
    """Verify plot_pattern supports laserhead, machine, and substrate views with expected aspect."""
    svg_path = tmp_path / "rect_10x5.svg"
    svg_path.write_text(_asymmetric_svg(10.0, 5.0), encoding="utf-8")

    eng = ExposureEngine(mode="vector", placement=SubstratePlacement(align="machine"))
    out_pat = tmp_path / "rect_10x5.pat"
    eng.generate(svg_path, output_path=out_pat)
    ptrn = eng.readbin()

    img_lh = eng.plot(ptrn, filename="test_view_lh", view="laserhead")
    img_mach = eng.plot(ptrn, filename="test_view_mach", view="machine")
    img_sub = eng.plot(ptrn, filename="test_view_sub", view="substrate")

    assert img_lh.ndim == 3 and np.any(img_lh > 0)
    assert img_mach.ndim == 3 and np.any(img_mach > 0)
    assert img_sub.ndim == 3 and np.any(img_sub > 0)

    # In align='machine' on 10x5 PCB: Laserhead u=5mm (narrow), v=10mm (tall),
    # while Machine / Substrate views are x=10mm (wide), y=5mm (short).
    lh_h, lh_w, _ = img_lh.shape
    mach_h, mach_w, _ = img_mach.shape
    sub_h, sub_w, _ = img_sub.shape
    assert lh_h > lh_w
    assert mach_w > mach_h
    assert sub_w > sub_h

    with pytest.raises(ValueError, match="Unsupported plot view"):
        plot_pattern(ptrn, cfg=eng.cfg, view="invalid_frame")
