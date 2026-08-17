import numpy as np
import pytest

from drill_rod_scanner.calibration.tilt_calibration import (
    calibrate_tilt,
    tilt_matrix,
)
from scripts.install_config import InstallConfig, rotation_matrix


def _side_mount_world_matrix():
    return np.column_stack([
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
    ])


def _simulate_scan(r_tilt: np.ndarray, n_lidar_points: int = 200, n_angles: int = 36) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(42)
    r_mount = rotation_matrix("z", 90.0)
    r_world = _side_mount_world_matrix()

    # A fixed set of physical points in the LiDAR scanning plane (z=0).
    points_lidar = []
    for _ in range(n_lidar_points):
        angle = rng.uniform(0, 2 * np.pi)
        distance = rng.uniform(0.5, 4.0)
        pts = [distance * np.cos(angle), distance * np.sin(angle), 0.0]
        points_lidar.append(pts)
    points_lidar = np.asarray(points_lidar, dtype=np.float64)

    angles_deg = np.linspace(0.0, 360.0, n_angles, endpoint=False)
    points_world = []
    angle_list = []
    for p_lidar in points_lidar:
        for theta in angles_deg:
            r_turntable = rotation_matrix("z", theta)
            p_world = p_lidar @ r_mount.T @ r_world.T @ r_turntable.T @ r_tilt.T
            points_world.append(p_world)
            angle_list.append(theta)
    return np.asarray(points_world), np.asarray(angle_list)


def test_tilt_matrix_matches_euler_sequence():
    roll, pitch, yaw = 1.0, -2.0, 0.5
    expected = rotation_matrix("z", yaw) @ rotation_matrix("y", pitch) @ rotation_matrix("x", roll)
    np.testing.assert_allclose(tilt_matrix(roll, pitch, yaw), expected, atol=1e-9)


def test_calibrate_tilt_recover_tilt():
    true_roll, true_pitch = 0.8, -1.2
    r_tilt = tilt_matrix(true_roll, true_pitch, 0.0)
    points, angles_deg = _simulate_scan(r_tilt, n_lidar_points=200, n_angles=36)

    roll, pitch, yaw = calibrate_tilt(
        points, angles_deg, voxel_size=0.10, max_distance=0.30, fix_yaw=True
    )
    assert abs(roll - true_roll) < 0.05
    assert abs(pitch - true_pitch) < 0.05
    assert abs(yaw) < 0.05


def test_calibrate_tilt_no_tilt():
    points, angles_deg = _simulate_scan(np.eye(3), n_lidar_points=200, n_angles=36)
    roll, pitch, yaw = calibrate_tilt(
        points, angles_deg, voxel_size=0.10, max_distance=0.30
    )
    assert abs(roll) < 0.02
    assert abs(pitch) < 0.02
    assert abs(yaw) < 0.02


def test_install_config_tilt_roundtrip(tmp_path):
    cfg = InstallConfig.side_mount()
    cfg.tilt_roll_deg = 0.12
    cfg.tilt_pitch_deg = -0.34
    cfg.tilt_yaw_deg = 0.05
    path = cfg.save(tmp_path / "install.yaml")
    loaded = InstallConfig.load(path)
    assert loaded.tilt_roll_deg == pytest.approx(0.12)
    assert loaded.tilt_pitch_deg == pytest.approx(-0.34)
    assert loaded.tilt_yaw_deg == pytest.approx(0.05)
    np.testing.assert_allclose(loaded.tilt_matrix(), cfg.tilt_matrix(), atol=1e-9)


def test_calibrate_tilt_not_enough_points_raises():
    with pytest.raises((ValueError, RuntimeError)):
        calibrate_tilt(np.empty((0, 3)), np.empty((0,)))
