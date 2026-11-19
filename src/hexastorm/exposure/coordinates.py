"""Explicit 3-frame coordinate transforms for Hexastorm laser exposure.

Defines the physical relationship between three right-handed 2D frames (in mm):

1. **Substrate frame** ``(x_sub, y_sub)``: Artwork / PCB design coordinates, with
   ``(0, 0)`` at bottom-left, ``+x_sub`` right, and ``+y_sub`` up.
2. **Machine frame** ``(x_mach, y_mach)``: CNC stage coordinates on the platform bed.
   With ``orth2lsrline = "x"``, ``+x_mach`` is gantry motion along each exposed lane
   and ``+y_mach`` is bed stepping between lanes.
3. **Laserhead frame** ``(u_scan, v_step)``: Optical scanhead coordinates, where
   ``u_scan`` is the fast prism scan axis (+ multi-lane stitching) and ``v_step``
   is the slow stage stepping axis along a lane.
"""

import math
from dataclasses import dataclass

import numpy as np

from ..config import PlatformConfig


def _cos_sin_deg(angle_deg: float) -> tuple[float, float]:
    """Return exact ``(cos, sin)`` snapping cardinal multiples of 90 degrees."""
    norm = angle_deg % 360.0
    rounded_q = round(norm / 90.0)
    if abs(norm - rounded_q * 90.0) < 1e-12:
        q = rounded_q % 4
        return ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))[q]
    rad = math.radians(angle_deg)
    return math.cos(rad), math.sin(rad)


def _rot_mat(angle_deg: float) -> np.ndarray:
    c, s = _cos_sin_deg(angle_deg)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _trans_mat(tx: float, ty: float) -> np.ndarray:
    return np.array(
        [[1.0, 0.0, float(tx)], [0.0, 1.0, float(ty)], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _flip_box_mat(
    mirror_x: bool, mirror_y: bool, width: float, height: float
) -> np.ndarray:
    sx = -1.0 if mirror_x else 1.0
    tx = float(width) if mirror_x else 0.0
    sy = -1.0 if mirror_y else 1.0
    ty = float(height) if mirror_y else 0.0
    return np.array(
        [[sx, 0.0, tx], [0.0, sy, ty], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _invert_affine(mat: np.ndarray) -> np.ndarray:
    """Invert a 3x3 homogeneous 2D affine matrix analytically."""
    a, b, tx = float(mat[0, 0]), float(mat[0, 1]), float(mat[0, 2])
    c, d, ty = float(mat[1, 0]), float(mat[1, 1]), float(mat[1, 2])
    det = a * d - b * c
    if abs(det) < 1e-15:
        raise ValueError("Singular coordinate transform matrix")
    inv_det = 1.0 / det
    ia = d * inv_det
    ib = -b * inv_det
    ic = -c * inv_det
    id_ = a * inv_det
    return np.array(
        [
            [ia, ib, -(ia * tx + ib * ty)],
            [ic, id_, -(ic * tx + id_ * ty)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _apply_affine(mat: np.ndarray, pts: np.ndarray) -> np.ndarray:
    arr = np.asarray(pts, dtype=np.float64)
    if arr.ndim == 1:
        if arr.size != 2:
            raise ValueError(f"Expected 2D point, got shape {arr.shape}")
        x = mat[0, 0] * arr[0] + mat[0, 1] * arr[1] + mat[0, 2]
        y = mat[1, 0] * arr[0] + mat[1, 1] * arr[1] + mat[1, 2]
        return np.array([x, y], dtype=np.float64)
    x = mat[0, 0] * arr[:, 0] + mat[0, 1] * arr[:, 1] + mat[0, 2]
    y = mat[1, 0] * arr[:, 0] + mat[1, 1] * arr[:, 1] + mat[1, 2]
    return np.column_stack((x, y))


def _apply_affine_xy(
    mat: np.ndarray, x: np.ndarray | float, y: np.ndarray | float
) -> tuple[np.ndarray, np.ndarray]:
    x_arr = np.asarray(x, dtype=np.float64)
    y_arr = np.asarray(y, dtype=np.float64)
    x_out = mat[0, 0] * x_arr + mat[0, 1] * y_arr + mat[0, 2]
    y_out = mat[1, 0] * x_arr + mat[1, 1] * y_arr + mat[1, 2]
    return x_out, y_out


@dataclass(frozen=True)
class MachineFrame:
    """Physical stage axis assignment and travel limits for the CNC machine."""

    step_axis: str = "x"
    x_size_mm: float = 200.0
    y_size_mm: float = 200.0

    def __post_init__(self) -> None:
        if self.step_axis not in ("x", "y"):
            raise ValueError(
                f"Unsupported step_axis {self.step_axis!r}; expected 'x' or 'y'"
            )
        if self.x_size_mm <= 0 or self.y_size_mm <= 0:
            raise ValueError("Machine x_size_mm and y_size_mm must be positive")

    @property
    def lane_axis(self) -> str:
        """Stage axis stepped between adjacent laser scan lanes."""
        return "y" if self.step_axis == "x" else "x"

    @property
    def head_rotation_deg(self) -> float:
        """Nominal CCW rotation from Machine ``(x, y)`` to Laserhead ``(u, v)``."""
        return 90.0 if self.step_axis == "x" else 0.0

    @classmethod
    def from_config(cls, cfg: PlatformConfig) -> "MachineFrame":
        """Build a :class:`MachineFrame` from ``cfg.motor_cfg``."""
        mtr = cfg.motor_cfg
        step_axis = str(mtr["orth2lsrline"]).lower()
        x_size = float(mtr["max_mm"]["x"] - mtr["min_mm"]["x"])
        y_size = float(mtr["max_mm"]["y"] - mtr["min_mm"]["y"])
        return cls(step_axis=step_axis, x_size_mm=x_size, y_size_mm=y_size)


@dataclass(frozen=True)
class SubstratePlacement:
    """Placement of the substrate design relative to Machine or Laserhead axes.

    Args:
        align: ``"laserhead"`` (default, preserves current Hexastorm orientation
            where substrate ``x`` maps to ``u_scan`` / Machine-Y and substrate ``y``
            maps to ``v_step`` / Machine-X) or ``"machine"`` (aligns substrate
            ``x`` directly with Machine-X gantry and ``y`` with Machine-Y bed).
        rotation_deg: Additional CCW rotation applied to the substrate [deg].
        mirror_x: Mirror the substrate along its local ``x`` axis.
        mirror_y: Mirror the substrate along its local ``y`` axis.
        origin_mm: Optional ``(x_mach, y_mach)`` bed offset on the machine [mm].
    """

    align: str = "laserhead"
    rotation_deg: float = 0.0
    mirror_x: bool = False
    mirror_y: bool = False
    origin_mm: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        if self.align not in ("laserhead", "machine"):
            raise ValueError(
                f"Unsupported align {self.align!r}; expected 'laserhead' or 'machine'"
            )
        if len(self.origin_mm) != 2:
            raise ValueError("origin_mm must be a 2-tuple (x_mm, y_mm)")


@dataclass
class LaserheadMount:
    """Explicit coordinate transform between Scanner ``(u, v)`` and optical hardware.

    Args:
        rotation_deg: Counter-clockwise mount rotation relative to the nominal head
            orientation [deg]. ``0.0`` keeps the default nominal mapping. ``90.0``
            corresponds to mounting the laserhead rotated 90 degrees CCW.
        mirror_u: Mirror along the prism scan axis ``u_scan``.
        mirror_v: Mirror along the stage step axis ``v_step``.
        invert_hw_scan_dir: If ``True`` (Hexastorm standard hardware sweep), clock
            tick ``0`` starts at the ``u_min`` side of each lane (legacy pixel
            ``bitsinscanline - 1``) and sweeps toward ``u_max`` (pixel ``0``).
    """

    rotation_deg: float = 0.0
    mirror_u: bool = False
    mirror_v: bool = False
    invert_hw_scan_dir: bool = True

    def _build_matrix(
        self, width_mm: float, height_mm: float
    ) -> tuple[np.ndarray, float, float]:
        rot = _rot_mat(self.rotation_deg)
        corners = np.array(
            [[0.0, 0.0], [width_mm, 0.0], [width_mm, height_mm], [0.0, height_mm]],
            dtype=np.float64,
        )
        rot_corners = _apply_affine(rot, corners)
        u_min, v_min = rot_corners.min(axis=0)
        u_max, v_max = rot_corners.max(axis=0)
        u_size = float(u_max - u_min)
        v_size = float(v_max - v_min)
        mat = (
            _flip_box_mat(self.mirror_u, self.mirror_v, u_size, v_size)
            @ _trans_mat(-float(u_min), -float(v_min))
            @ rot
        )
        return mat, u_size, v_size

    def transform_bounds(
        self, width_mm: float, height_mm: float
    ) -> tuple[float, float, float, float]:
        """Return ``(u_min, v_min, u_size, v_size)`` after rotating the design box."""
        rot = _rot_mat(self.rotation_deg)
        corners = np.array(
            [[0.0, 0.0], [width_mm, 0.0], [width_mm, height_mm], [0.0, height_mm]],
            dtype=np.float64,
        )
        rot_corners = _apply_affine(rot, corners)
        u_min, v_min = rot_corners.min(axis=0)
        u_max, v_max = rot_corners.max(axis=0)
        return float(u_min), float(v_min), float(u_max - u_min), float(v_max - v_min)

    def transform_points(
        self, pts: np.ndarray, width_mm: float, height_mm: float
    ) -> tuple[np.ndarray, float, float]:
        """Transform ``(K, 2)`` coordinates [mm] into scanner ``(u, v)`` [mm]."""
        mat, u_size, v_size = self._build_matrix(width_mm, height_mm)
        return _apply_affine(mat, pts), u_size, v_size

    def inverse_transform_points(
        self, uv_pts: np.ndarray, width_mm: float, height_mm: float
    ) -> np.ndarray:
        """Inverse of :meth:`transform_points`: scanner ``(u, v)`` back to input ``(x, y)``."""
        mat, _u_size, _v_size = self._build_matrix(width_mm, height_mm)
        return _apply_affine(_invert_affine(mat), uv_pts)


class CoordinateTransform:
    """Composed forward/inverse 2D affine transforms across Substrate, Machine, Laserhead."""

    def __init__(
        self,
        width_mm: float,
        height_mm: float,
        machine: MachineFrame | None = None,
        placement: SubstratePlacement | None = None,
        mount: LaserheadMount | None = None,
    ):
        if width_mm <= 0 or height_mm <= 0:
            raise ValueError("Sample width_mm and height_mm must be positive")

        self.width_mm = float(width_mm)
        self.height_mm = float(height_mm)
        self.machine = machine if machine is not None else MachineFrame()
        self.placement = placement if placement is not None else SubstratePlacement()
        self.mount = mount if mount is not None else LaserheadMount()

        sub_corners = np.array(
            [
                [0.0, 0.0],
                [self.width_mm, 0.0],
                [self.width_mm, self.height_mm],
                [0.0, self.height_mm],
            ],
            dtype=np.float64,
        )

        # 1. Substrate -> Machine (local bed box starting at (0, 0))
        alpha_deg = (
            self.placement.rotation_deg
            if self.placement.align == "machine"
            else self.placement.rotation_deg - self.machine.head_rotation_deg
        )
        sub_to_mach_pre = _rot_mat(alpha_deg) @ _flip_box_mat(
            self.placement.mirror_x,
            self.placement.mirror_y,
            self.width_mm,
            self.height_mm,
        )
        mach_corners_pre = _apply_affine(sub_to_mach_pre, sub_corners)
        min_mx, min_my = mach_corners_pre.min(axis=0)
        max_mx, max_my = mach_corners_pre.max(axis=0)
        self.machine_width_mm = float(max_mx - min_mx)
        self.machine_height_mm = float(max_my - min_my)
        self.sub_to_mach_matrix = (
            _trans_mat(-float(min_mx), -float(min_my)) @ sub_to_mach_pre
        )
        self.mach_to_sub_matrix = _invert_affine(self.sub_to_mach_matrix)

        # 2. Machine -> Laserhead (bounds computed from the 4 actual substrate corners)
        beta_deg = self.machine.head_rotation_deg + self.mount.rotation_deg
        mach_corners = _apply_affine(self.sub_to_mach_matrix, sub_corners)
        rot_lh = _rot_mat(beta_deg)
        lh_corners_pre = _apply_affine(rot_lh, mach_corners)
        min_u, min_v = lh_corners_pre.min(axis=0)
        max_u, max_v = lh_corners_pre.max(axis=0)
        self.u_size_mm = float(max_u - min_u)
        self.v_size_mm = float(max_v - min_v)

        self.mach_to_lh_matrix = (
            _flip_box_mat(
                self.mount.mirror_u,
                self.mount.mirror_v,
                self.u_size_mm,
                self.v_size_mm,
            )
            @ _trans_mat(-float(min_u), -float(min_v))
            @ rot_lh
        )
        self.lh_to_mach_matrix = _invert_affine(self.mach_to_lh_matrix)

        # 3. Direct composition Substrate <-> Laserhead
        self.sub_to_lh_matrix = self.mach_to_lh_matrix @ self.sub_to_mach_matrix
        self.lh_to_sub_matrix = _invert_affine(self.sub_to_lh_matrix)
        self.is_identity_lh_to_sub = bool(
            np.allclose(self.lh_to_sub_matrix, np.eye(3, dtype=np.float64), atol=1e-12)
        )

    def substrate_to_machine(
        self, pts: np.ndarray, include_origin: bool = False
    ) -> np.ndarray:
        """Transform Substrate ``(x, y)`` to Machine ``(x_mach, y_mach)`` [mm]."""
        out = _apply_affine(self.sub_to_mach_matrix, pts)
        if include_origin:
            ox, oy = self.placement.origin_mm
            out = out + np.array([ox, oy], dtype=np.float64)
        return out

    def machine_to_substrate(
        self, pts: np.ndarray, include_origin: bool = False
    ) -> np.ndarray:
        """Transform Machine ``(x_mach, y_mach)`` to Substrate ``(x, y)`` [mm]."""
        arr = np.asarray(pts, dtype=np.float64)
        if include_origin:
            ox, oy = self.placement.origin_mm
            arr = arr - np.array([ox, oy], dtype=np.float64)
        return _apply_affine(self.mach_to_sub_matrix, arr)

    def machine_to_laserhead(self, pts: np.ndarray) -> np.ndarray:
        """Transform local Machine ``(x_mach, y_mach)`` to Laserhead ``(u_scan, v_step)`` [mm]."""
        return _apply_affine(self.mach_to_lh_matrix, pts)

    def laserhead_to_machine(self, pts: np.ndarray) -> np.ndarray:
        """Transform Laserhead ``(u_scan, v_step)`` to local Machine ``(x_mach, y_mach)`` [mm]."""
        return _apply_affine(self.lh_to_mach_matrix, pts)

    def substrate_to_laserhead(self, pts: np.ndarray) -> np.ndarray:
        """Transform Substrate ``(x, y)`` directly to Laserhead ``(u_scan, v_step)`` [mm]."""
        return _apply_affine(self.sub_to_lh_matrix, pts)

    def laserhead_to_substrate(self, pts: np.ndarray) -> np.ndarray:
        """Transform Laserhead ``(u_scan, v_step)`` directly to Substrate ``(x, y)`` [mm]."""
        return _apply_affine(self.lh_to_sub_matrix, pts)

    def laserhead_to_substrate_xy(
        self, u_mm: np.ndarray | float, v_mm: np.ndarray | float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized Laserhead ``(u_mm, v_mm)`` -> Substrate ``(x_mm, y_mm)``."""
        return _apply_affine_xy(self.lh_to_sub_matrix, u_mm, v_mm)

    def laserhead_to_machine_xy(
        self, u_mm: np.ndarray | float, v_mm: np.ndarray | float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized Laserhead ``(u_mm, v_mm)`` -> Machine ``(x_mach_mm, y_mach_mm)``."""
        return _apply_affine_xy(self.lh_to_mach_matrix, u_mm, v_mm)

    def validate_fits_machine(
        self,
        max_x_mm: float | None = None,
        max_y_mm: float | None = None,
    ) -> None:
        """Validate placed substrate fits inside physical machine bed limits."""
        limit_x = self.machine.x_size_mm if max_x_mm is None else float(max_x_mm)
        limit_y = self.machine.y_size_mm if max_y_mm is None else float(max_y_mm)
        ox, oy = float(self.placement.origin_mm[0]), float(self.placement.origin_mm[1])
        if (
            ox < -1e-9
            or oy < -1e-9
            or (ox + self.machine_width_mm) > (limit_x + 1e-9)
            or (oy + self.machine_height_mm) > (limit_y + 1e-9)
        ):
            raise ValueError("Object does not fit on platform")
