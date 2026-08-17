#!/usr/bin/env python3
"""Verify joint tilt + LiDAR tilt calibration against a known ground-truth config.

Usage:
    python scripts/verify_tilt_calibration.py \
        --config configs/install_side_mount.yaml \
        --voxel-size 0.05

The script:
1. Loads the YAML config and uses its ``tilt`` and ``lidar_tilt`` values as ground truth.
2. Simulates a 360° scan of a room with those tilts applied.
3. Runs ``joint_tilt_calibration.calibrate_tilt`` on the simulated data.
4. Prints ground truth, estimated values, and errors.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from install_config import InstallConfig, rotation_matrix  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "drill_rod_scanner" / "calibration"))
from joint_tilt_calibration import (  # noqa: E402
    calibrate_tilt,
    lidar_tilt_matrix,
    tilt_matrix,
)


def _simulate_room_scan(
    cfg: InstallConfig,
    n_ground_pairs: int = 400,
    n_wall_points: int = 40,
    noise_m: float = 0.0005,
) -> tuple[np.ndarray, np.ndarray]:
    """Simulate a 360° scan using the tilts stored in *cfg* as ground truth."""
    rng = np.random.default_rng(42)
    m = cfg.mount_matrix().T @ cfg.to_world_matrix()
    r_tilt = tilt_matrix(cfg.tilt_roll_deg, cfg.tilt_pitch_deg, cfg.tilt_yaw_deg)
    r_lidar = lidar_tilt_matrix(
        cfg.lidar_tilt_roll_deg, cfg.lidar_tilt_pitch_deg, cfg.lidar_tilt_yaw_deg
    )

    physical_points: list[np.ndarray] = []
    # Ground plane (z=0), with 180° counterparts for symmetry.
    for _ in range(n_ground_pairs):
        x = rng.uniform(-3.0, 3.0)
        y = rng.uniform(-3.0, 3.0)
        physical_points.append(np.array([x, y, 0.0]))
        physical_points.append(np.array([-x, -y, 0.0]))
    # Four walls, symmetric by construction.
    for _ in range(n_wall_points):
        z = rng.uniform(0.0, 2.0)
        physical_points.append(np.array([rng.uniform(-3.0, 3.0), -2.0, z]))
        physical_points.append(np.array([rng.uniform(-3.0, 3.0), 2.0, z]))
        physical_points.append(np.array([-2.0, rng.uniform(-2.0, 2.0), z]))
        physical_points.append(np.array([2.0, rng.uniform(-2.0, 2.0), z]))
    # Asymmetric corner: breaks 180° rotational symmetry so global yaw (lidar roll)
    # becomes observable from the symmetry residual.
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
        # Angles where the vertical scanning plane contains the point.
        theta1 = np.degrees(np.arctan2(-p[0], p[1]))
        theta2 = (theta1 + 180.0) % 360.0
        for theta in [theta1, theta2]:
            r_theta = rotation_matrix("z", theta)
            p_computed = (
                p @ r_tilt @ r_theta @ r_tilt.T
                @ m.T @ r_lidar.T @ m @ r_theta.T
            )
            p_computed += rng.normal(scale=noise_m, size=3)
            measured.append(p_computed)
            angles.append(theta)

    return np.asarray(measured, dtype=np.float64), np.asarray(angles, dtype=np.float64)


def _print_errors(name: str, truth: tuple[float, ...], est: tuple[float, ...]) -> None:
    print(f"\n{name}")
    labels = ("roll", "pitch", "yaw")
    for label, t, e in zip(labels, truth, est):
        print(f"  {label:5s}: truth={t:+.6f}°  est={e:+.6f}°  err={abs(e - t):.6f}°")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/install_side_mount.yaml",
        help="Ground-truth install config YAML (default: configs/install_side_mount.yaml)",
    )
    parser.add_argument("--voxel-size", type=float, default=0.05, help=" downsampling size (m)")
    args = parser.parse_args()

    cfg = InstallConfig.load(args.config)
    print(f"[verify] loaded ground-truth config: {args.config}")
    print(f"[verify] tilt       : roll={cfg.tilt_roll_deg} pitch={cfg.tilt_pitch_deg} yaw={cfg.tilt_yaw_deg}")
    print(f"[verify] lidar_tilt : roll={cfg.lidar_tilt_roll_deg} pitch={cfg.lidar_tilt_pitch_deg} yaw={cfg.lidar_tilt_yaw_deg}")

    points, angles_deg = _simulate_room_scan(cfg)
    print(f"[verify] simulated {points.shape[0]} points")

    (tt_roll, tt_pitch, tt_yaw), (l_roll, l_pitch, l_yaw) = calibrate_tilt(
        points, angles_deg, voxel_size=args.voxel_size
    )

    _print_errors(
        "Turntable-axis tilt",
        (cfg.tilt_roll_deg, cfg.tilt_pitch_deg, cfg.tilt_yaw_deg),
        (tt_roll, tt_pitch, tt_yaw),
    )
    _print_errors(
        "LiDAR mounting tilt",
        (cfg.lidar_tilt_roll_deg, cfg.lidar_tilt_pitch_deg, cfg.lidar_tilt_yaw_deg),
        (l_roll, l_pitch, l_yaw),
    )


if __name__ == "__main__":
    main()
