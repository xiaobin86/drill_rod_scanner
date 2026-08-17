"""360° self-calibration for turntable-axis tilt.

A full 360° scan of a static scene is approximately invariant under a 180°
rotation about the actual turntable axis. This module finds that axis and
returns the roll/pitch/yaw angles needed to align it with the world z-axis.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def _rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    if axis == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    raise ValueError(f"unsupported axis: {axis}")


def tilt_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return R_tilt = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    This is the rotation that maps the actual turntable axis to the world z-axis.
    Its inverse is applied to point clouds after turntable rotation.
    """
    rx = _rotation_matrix("x", roll_deg)
    ry = _rotation_matrix("y", pitch_deg)
    rz = _rotation_matrix("z", yaw_deg)
    return rz @ ry @ rx


def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points
    voxel = np.floor(points / voxel_size).astype(np.int64)
    groups: dict[tuple[int, int, int], list[np.ndarray]] = {}
    for key, pt in zip(map(tuple, voxel), points):
        groups.setdefault(key, []).append(pt)
    return np.array([np.mean(pts, axis=0) for pts in groups.values()])


def _rotation_about_axis(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    kx, ky, kz = k
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def _nearest_neighbor_dists(points: np.ndarray, query: np.ndarray) -> np.ndarray:
    tree = cKDTree(points)
    return tree.query(query, k=1)[0]


def _axis_from_tilt_angles(tilt_x_deg: float, tilt_y_deg: float) -> np.ndarray:
    """Return actual turntable axis from small tilt angles about x and y."""
    tx = np.deg2rad(tilt_x_deg)
    ty = np.deg2rad(tilt_y_deg)
    axis = np.array([np.sin(ty), -np.sin(tx), np.cos(tx) * np.cos(ty)])
    return axis / np.linalg.norm(axis)


def _tilt_angles_from_axis(axis: np.ndarray) -> tuple[float, float]:
    """Return (tilt_x, tilt_y) such that z rotated by Rx(-tilt_x)Ry(-tilt_y) aligns with axis."""
    axis = axis / np.linalg.norm(axis)
    tilt_x = -np.degrees(np.arctan2(axis[1], axis[2]))
    z_rem = np.hypot(axis[1], axis[2])
    tilt_y = np.degrees(np.arctan2(axis[0], z_rem))
    return float(tilt_x), float(tilt_y)


def calibrate_tilt(
    points: np.ndarray,
    angles_deg: np.ndarray | None = None,
    voxel_size: float = 0.02,
    max_distance: float = 0.05,
    fix_yaw: bool = True,
) -> tuple[float, float, float]:
    """Estimate roll/pitch/yaw tilt from a 360° scan.

    Args:
        points: (N, 3) point cloud in the world frame after ideal mounting and
            turntable rotation.
        angles_deg: kept for API compatibility; not used by this symmetry-based
            method.
        voxel_size: downsampling grid size in meters.
        max_distance: not used; kept for API compatibility.
        fix_yaw: if True, return yaw=0 (turntable axis tilt has only two DOF).

    Returns:
        (roll_deg, pitch_deg, yaw_deg) estimated tilt angles.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be (N, 3), got {points.shape}")

    cloud = _voxel_downsample(points, voxel_size)
    if cloud.shape[0] < 10:
        raise RuntimeError("not enough points after downsampling")

    def residuals(params: np.ndarray) -> np.ndarray:
        axis = _axis_from_tilt_angles(params[0], params[1])
        r_180 = _rotation_about_axis(axis, 180.0)
        rotated = cloud @ r_180.T
        return np.sqrt(_nearest_neighbor_dists(cloud, rotated))

    result = least_squares(
        residuals,
        np.array([0.0, 0.0]),
        method="lm",
        max_nfev=200,
    )
    if not result.success:
        raise RuntimeError(f"optimization failed: {result.message}")

    tilt_x, tilt_y = result.x
    roll, pitch = _tilt_angles_from_axis(_axis_from_tilt_angles(tilt_x, tilt_y))
    yaw = 0.0 if fix_yaw else 0.0
    return float(roll), float(pitch), float(yaw)
