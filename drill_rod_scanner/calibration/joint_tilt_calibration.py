"""Joint 360° self-calibration for turntable-axis tilt and LiDAR mounting tilt.

The physically correct correction model is:

    p_corrected = p_measured
        @ R_z(theta)                       # undo ideal turntable rotation
        @ M.T @ R_lidar @ M                # undo LiDAR mounting deviation (world frame)
        @ R_tilt @ R_z(theta).T @ R_tilt.T # redo rotation around actual turntable axis

where R_tilt aligns the actual turntable axis with world z, and R_lidar is the
LiDAR mounting deviation (ideal radar -> actual radar).
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares, minimize
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


def _axis_from_tilt_angles(tilt_x_deg: float, tilt_y_deg: float) -> np.ndarray:
    """Return actual turntable axis from small tilt angles about x and y."""
    tx = np.deg2rad(tilt_x_deg)
    ty = np.deg2rad(tilt_y_deg)
    axis = np.array([
        np.sin(ty) * np.cos(tx),
        np.sin(tx) * np.cos(ty),
        np.cos(tx) * np.cos(ty),
    ])
    return axis / np.linalg.norm(axis)


def _tilt_angles_from_axis(axis: np.ndarray) -> tuple[float, float]:
    """Return (tilt_x, tilt_y) such that R_align @ z = axis."""
    axis = axis / np.linalg.norm(axis)
    tilt_x = np.degrees(np.arctan2(axis[1], axis[2]))
    tilt_y = np.degrees(np.arctan2(axis[0], axis[2]))
    return float(tilt_x), float(tilt_y)


def _align_axis_to_z(axis: np.ndarray) -> np.ndarray:
    """Return rotation matrix R (column-vector convention) such that R @ z = axis."""
    axis = axis / np.linalg.norm(axis)
    z = np.array([0.0, 0.0, 1.0])
    cross = np.cross(axis, z)
    cross_norm = np.linalg.norm(cross)
    if cross_norm < 1e-12:
        return np.eye(3)
    rot_axis = cross / cross_norm
    angle = np.arccos(np.clip(np.dot(axis, z), -1.0, 1.0))
    k = rot_axis
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def tilt_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return turntable-axis alignment matrix (column-vector convention).

    The returned matrix R satisfies ``R @ z`` = actual turntable axis direction.
    For small roll/pitch the axis is approximately ``[pitch, roll, 1]``.
    """
    axis = _axis_from_tilt_angles(roll_deg, pitch_deg)
    r_align = _align_axis_to_z(axis).T
    rz = _rotation_matrix("z", yaw_deg)
    return r_align @ rz


def lidar_tilt_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return LiDAR mounting deviation matrix (ideal radar -> actual radar)."""
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


def _nearest_neighbor_dists(points: np.ndarray, query: np.ndarray) -> np.ndarray:
    tree = cKDTree(points)
    return tree.query(query, k=1)[0]


def _corrected_cloud(
    points: np.ndarray,
    angles_deg: np.ndarray,
    r_tilt: np.ndarray,
    m: np.ndarray,
    r_lidar_tilt: np.ndarray,
) -> np.ndarray:
    """Return points corrected using estimated tilts."""
    corrected = np.empty_like(points)
    for angle in np.unique(angles_deg):
        mask = np.isclose(angles_deg, angle)
        r_theta = _rotation_matrix("z", angle)
        corrected[mask] = (
            points[mask] @ r_theta
            @ m.T @ r_lidar_tilt @ m
            @ r_tilt @ r_theta.T @ r_tilt.T
        )
    return corrected


def _symmetry_residuals(
    cloud: np.ndarray,
    cloud_angles: np.ndarray,
    r_tilt: np.ndarray,
    m: np.ndarray,
    r_lidar_tilt: np.ndarray,
) -> np.ndarray:
    """Return residual vector for 180° rotational symmetry about world z.

    For each point measured at angle theta, find the nearest corrected point
    measured at theta+180°. Their distance should be zero for a calibrated scan.
    """
    corrected = _corrected_cloud(cloud, cloud_angles, r_tilt, m, r_lidar_tilt)
    a = np.asarray(cloud_angles, dtype=np.float64) % 360.0
    residuals: list[float] = []
    for angle in np.unique(a):
        opposite = (angle + 180.0) % 360.0
        mask_src = np.isclose(a, angle)
        mask_dst = np.isclose(a, opposite)
        if not mask_src.any() or not mask_dst.any():
            continue
        src = corrected[mask_src]
        dst = corrected[mask_dst]
        dists = _nearest_neighbor_dists(dst, src)
        residuals.extend(dists.tolist())
    if not residuals:
        return np.array([0.0])
    return np.asarray(residuals, dtype=np.float64)


def _ground_flatness_cost(corrected: np.ndarray, percentile: float = 30.0) -> float:
    """Return standard deviation of z for the lowest `percentile` points."""
    if corrected.shape[0] < 10:
        return float("inf")
    z_threshold = np.percentile(corrected[:, 2], percentile)
    ground = corrected[corrected[:, 2] <= z_threshold]
    if ground.shape[0] < 3:
        return float("inf")
    return float(ground[:, 2].std())


def _calibrate_turntable_tilt(
    points: np.ndarray,
    angles_deg: np.ndarray,
    m: np.ndarray,
    voxel_size: float,
    r_lidar_tilt: np.ndarray | None = None,
) -> tuple[float, float]:
    """Estimate turntable axis roll/pitch from 180° rotational symmetry."""
    cloud = _voxel_downsample(points, voxel_size)
    tree = cKDTree(points)
    _, indices = tree.query(cloud, k=1)
    cloud_angles = angles_deg[indices]
    if r_lidar_tilt is None:
        r_lidar_tilt = np.eye(3)

    def total_cost(params: np.ndarray) -> float:
        r_tilt = tilt_matrix(params[0], params[1], 0.0)
        sym = _symmetry_residuals(cloud, cloud_angles, r_tilt, m, r_lidar_tilt)
        flat = _ground_flatness_cost(
            _corrected_cloud(cloud, cloud_angles, r_tilt, m, r_lidar_tilt)
        )
        return float(np.sum(sym ** 2)) + 1e4 * flat

    best_cost = float("inf")
    best_params = np.zeros(2)
    for r0 in np.linspace(-3.0, 3.0, 9):
        for p0 in np.linspace(-3.0, 3.0, 9):
            c = total_cost(np.array([r0, p0]))
            if c < best_cost:
                best_cost = c
                best_params = np.array([r0, p0])

    result = minimize(
        total_cost,
        best_params,
        method="L-BFGS-B",
        bounds=[(-10.0, 10.0), (-10.0, 10.0)],
        options={"maxiter": 200},
    )
    if not result.success:
        raise RuntimeError(f"turntable tilt optimization failed: {result.message}")
    return float(result.x[0]), float(result.x[1])


def _calibrate_lidar_tilt(
    points: np.ndarray,
    angles_deg: np.ndarray,
    r_tilt: np.ndarray,
    m: np.ndarray,
    voxel_size: float,
) -> tuple[float, float, float]:
    """Estimate LiDAR mounting yaw.

    In the side-mount convention, only yaw (twist around radar z) is reliably
    observable from a single 360° scan via ground-plane flatness. roll/pitch
    are fixed to 0.
    """
    cloud = _voxel_downsample(points, voxel_size)
    tree = cKDTree(points)
    _, indices = tree.query(cloud, k=1)
    cloud_angles = angles_deg[indices]

    def cost(yaw_arr: np.ndarray) -> float:
        yaw = float(np.asarray(yaw_arr).reshape(-1)[0])
        r_lidar_tilt = lidar_tilt_matrix(0.0, 0.0, yaw)
        corrected = _corrected_cloud(cloud, cloud_angles, r_tilt, m, r_lidar_tilt)
        return _ground_flatness_cost(corrected)

    best_cost = float("inf")
    best_yaw = 0.0
    for yaw0 in np.linspace(-5.0, 5.0, 51):
        c = cost(np.array([yaw0]))
        if c < best_cost:
            best_cost = c
            best_yaw = yaw0

    result = minimize(
        cost,
        np.array([best_yaw]),
        method="L-BFGS-B",
        bounds=[(-10.0, 10.0)],
        options={"maxiter": 200},
    )
    if result.success and result.fun < best_cost:
        best_yaw = float(result.x[0])

    return 0.0, 0.0, best_yaw


def calibrate_tilt(
    points: np.ndarray,
    angles_deg: np.ndarray,
    m: np.ndarray | None = None,
    voxel_size: float = 0.02,
    max_distance: float = 0.05,
    calibrate_lidar_tilt: bool = True,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Estimate turntable-axis tilt and LiDAR mounting tilt from a 360° scan.

    Args:
        points: (N, 3) point cloud computed with ideal mounting and turntable
            model (i.e., the output of servo_sweep_scan.py when tilt configs
            are zero).
        angles_deg: (N,) turntable angle for each point.
        m: (3, 3) row-vector transformation from ideal radar frame at theta=0
            to world frame: ``p_world = p_radar @ m @ Rz(theta).T``. If None,
            the side-mount default is used.
        voxel_size: downsampling grid size in meters.
        max_distance: not used; kept for API compatibility.
        calibrate_lidar_tilt: if True, also estimate LiDAR mounting tilt.

    Returns:
        ((turntable_roll, turntable_pitch, turntable_yaw),
         (lidar_roll, lidar_pitch, lidar_yaw)) in degrees.
    """
    points = np.asarray(points, dtype=np.float64)
    angles_deg = np.asarray(angles_deg, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be (N, 3), got {points.shape}")
    if angles_deg.shape[0] != points.shape[0]:
        raise ValueError("angles_deg length must match points row count")

    if m is None:
        m = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

    # Estimate LiDAR yaw first (ground flatness is weakly coupled with turntable
    # tilt), then alternate with turntable-axis tilt refinement.
    tt_roll, tt_pitch = 0.0, 0.0
    r_tilt = np.eye(3)
    l_roll = l_pitch = 0.0
    l_yaw = 0.0

    if calibrate_lidar_tilt:
        _, _, l_yaw = _calibrate_lidar_tilt(points, angles_deg, r_tilt, m, voxel_size)

    for _ in range(3):
        r_lidar_tilt = lidar_tilt_matrix(l_roll, l_pitch, l_yaw)
        tt_roll, tt_pitch = _calibrate_turntable_tilt(
            points, angles_deg, m, voxel_size, r_lidar_tilt=r_lidar_tilt
        )
        r_tilt = tilt_matrix(tt_roll, tt_pitch, 0.0)
        if calibrate_lidar_tilt:
            _, _, l_yaw = _calibrate_lidar_tilt(
                points, angles_deg, r_tilt, m, voxel_size
            )

    return (
        (float(tt_roll), float(tt_pitch), 0.0),
        (float(l_roll), float(l_pitch), float(l_yaw)),
    )
