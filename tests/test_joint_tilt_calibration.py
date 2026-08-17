import numpy as np
import pytest

from drill_rod_scanner.calibration.joint_tilt_calibration import (
    calibrate_tilt,
    lidar_tilt_matrix,
    tilt_matrix,
)
from scripts.install_config import InstallConfig, rotation_matrix


def _side_mount_config(**kwargs) -> InstallConfig:
    cfg = InstallConfig.side_mount()
    for key, value in kwargs.items():
        setattr(cfg, key, value)
    return cfg


def _simulate_room_scan(cfg: InstallConfig) -> tuple[np.ndarray, np.ndarray]:
    """Simulate a 360° scan using the tilts stored in *cfg* as ground truth."""
    rng = np.random.default_rng(42)
    m = cfg.mount_matrix().T @ cfg.to_world_matrix()
    r_tilt = tilt_matrix(cfg.tilt_roll_deg, cfg.tilt_pitch_deg, cfg.tilt_yaw_deg)
    r_lidar = lidar_tilt_matrix(
        cfg.lidar_tilt_roll_deg, cfg.lidar_tilt_pitch_deg, cfg.lidar_tilt_yaw_deg
    )

    physical_points: list[np.ndarray] = []
    for _ in range(400):
        x = rng.uniform(-3.0, 3.0)
        y = rng.uniform(-3.0, 3.0)
        physical_points.append(np.array([x, y, 0.0]))
        physical_points.append(np.array([-x, -y, 0.0]))
    for _ in range(40):
        z = rng.uniform(0.0, 2.0)
        physical_points.append(np.array([rng.uniform(-3.0, 3.0), -2.0, z]))
        physical_points.append(np.array([rng.uniform(-3.0, 3.0), 2.0, z]))
        physical_points.append(np.array([-2.0, rng.uniform(-2.0, 2.0), z]))
        physical_points.append(np.array([2.0, rng.uniform(-2.0, 2.0), z]))
    for _ in range(50):
        physical_points.append(np.array([
            1.0 + rng.uniform(-0.3, 0.3),
            1.0 + rng.uniform(-0.3, 0.3),
            rng.uniform(0.0, 1.5),
        ]))
    physical_points = np.asarray(physical_points, dtype=np.float64)

    measured: list[np.ndarray] = []
    angles: list[float] = []
    for p in physical_points:
        theta1 = np.degrees(np.arctan2(-p[0], p[1]))
        theta2 = (theta1 + 180.0) % 360.0
        for theta in [theta1, theta2]:
            r_theta = rotation_matrix("z", theta)
            p_computed = (
                p @ r_tilt @ r_theta @ r_tilt.T
                @ m.T @ r_lidar.T @ m @ r_theta.T
            )
            p_computed += rng.normal(scale=0.0005, size=3)
            measured.append(p_computed)
            angles.append(theta)

    return np.asarray(measured, dtype=np.float64), np.asarray(angles, dtype=np.float64)


@pytest.mark.parametrize("true_yaw", [-2.0, -1.0, 0.0, 1.0, 2.0])
def test_calibrate_lidar_yaw_only(true_yaw: float):
    cfg = _side_mount_config(
        tilt_roll_deg=0.0,
        tilt_pitch_deg=0.0,
        tilt_yaw_deg=0.0,
        lidar_tilt_roll_deg=0.0,
        lidar_tilt_pitch_deg=0.0,
        lidar_tilt_yaw_deg=true_yaw,
    )
    points, angles = _simulate_room_scan(cfg)
    (_, _, _), (_, _, est_yaw) = calibrate_tilt(points, angles, voxel_size=0.05)
    assert est_yaw == pytest.approx(true_yaw, abs=0.05)


def test_calibrate_turntable_tilt_only():
    cfg = _side_mount_config(
        tilt_roll_deg=2.0,
        tilt_pitch_deg=-1.0,
        tilt_yaw_deg=0.0,
        lidar_tilt_roll_deg=0.0,
        lidar_tilt_pitch_deg=0.0,
        lidar_tilt_yaw_deg=0.0,
    )
    points, angles = _simulate_room_scan(cfg)
    (est_roll, est_pitch, _), _ = calibrate_tilt(points, angles, voxel_size=0.05)
    assert est_roll == pytest.approx(2.0, abs=0.3)
    assert est_pitch == pytest.approx(-1.0, abs=0.1)


def test_calibrate_target_case_ignores_unobservable_lidar_roll():
    """lidar_tilt.roll is a global yaw and is fixed to 0 by the estimator."""
    cfg = _side_mount_config(
        tilt_roll_deg=0.0,
        tilt_pitch_deg=0.0,
        tilt_yaw_deg=0.0,
        lidar_tilt_roll_deg=1.5,
        lidar_tilt_pitch_deg=0.0,
        lidar_tilt_yaw_deg=1.5,
    )
    points, angles = _simulate_room_scan(cfg)
    ((tt_roll, tt_pitch, tt_yaw), (l_roll, l_pitch, l_yaw)) = calibrate_tilt(
        points, angles, voxel_size=0.05
    )
    assert tt_roll == pytest.approx(0.0, abs=0.05)
    assert tt_pitch == pytest.approx(0.0, abs=0.05)
    assert tt_yaw == pytest.approx(0.0, abs=0.05)
    assert l_roll == pytest.approx(0.0, abs=0.05)
    assert l_pitch == pytest.approx(0.0, abs=0.05)
    assert l_yaw == pytest.approx(1.5, abs=0.05)
