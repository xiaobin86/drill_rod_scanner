#!/usr/bin/env python3
"""Offline tilt calibration CLI.

Estimate roll/pitch/yaw from a saved 360° scan and write the readable angles
back to the original install config YAML.

Usage:
    python scripts/calibrate_tilt.py \
        --cloud output/scan_xxx/cloud.npy \
        --angles output/scan_xxx/angles.npy \
        --install-config configs/install_side_mount.yaml

If only a .npy point cloud is given without angles, the script assumes the
cloud was already split into front/back order and synthesizes 0-360 angles.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from install_config import InstallConfig  # noqa: E402
from drill_rod_scanner.calibration.tilt_calibration import calibrate_tilt  # noqa: E402


def load_cloud(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".npy":
        return np.load(path)
    if suffix in {".ply", ".pcd"}:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(path))
        return np.asarray(pcd.points)
    raise ValueError(f"unsupported cloud format: {suffix}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", type=Path, required=True, help="input point cloud (.npy/.ply/.pcd)")
    parser.add_argument("--angles", type=Path, default=None, help="per-point angles in degrees (.npy)")
    parser.add_argument("--install-config", type=Path, required=True, help="install config YAML to read and update")
    parser.add_argument("--voxel-size", type=float, default=0.02, help="downsampling voxel size (m)")
    parser.add_argument("--k", type=int, default=10, help="neighbors for local plane fitting")
    parser.add_argument("--max-distance", type=float, default=0.05, help="max correspondence distance (m)")
    parser.add_argument("--fix-yaw", action="store_true", help="constrain yaw to 0")
    parser.add_argument("--dry-run", action="store_true", help="print results without writing file")
    args = parser.parse_args()

    points = load_cloud(args.cloud)
    if points.ndim != 2 or points.shape[1] != 3:
        raise SystemExit(f"cloud must have shape (N, 3), got {points.shape}")

    if args.angles:
        angles_deg = np.load(args.angles)
    else:
        angles_deg = np.linspace(0.0, 360.0, points.shape[0], endpoint=False)

    if angles_deg.shape[0] != points.shape[0]:
        raise SystemExit("angles length must match point count")

    cfg = InstallConfig.load(args.install_config)
    print(f"[load] {points.shape[0]} points from {args.cloud}")
    print(f"[calibrate] voxel={args.voxel_size}m k={args.k} max_dist={args.max_distance}m fix_yaw={args.fix_yaw}")

    roll, pitch, yaw = calibrate_tilt(
        points,
        angles_deg,
        voxel_size=args.voxel_size,
        k=args.k,
        max_distance=args.max_distance,
        fix_yaw=args.fix_yaw,
    )

    print(f"[result] roll={roll:.6f}° pitch={pitch:.6f}° yaw={yaw:.6f}°")

    cfg.tilt_roll_deg = roll
    cfg.tilt_pitch_deg = pitch
    cfg.tilt_yaw_deg = yaw
    cfg.name = f"{cfg.name}-calibrated"
    cfg.description = f"{cfg.description} + 360° tilt calibration"

    if args.dry_run:
        print("[dry-run] not writing config")
        return

    cfg.save(args.install_config)
    print(f"[write] tilt angles written back to {args.install_config}")


if __name__ == "__main__":
    main()
