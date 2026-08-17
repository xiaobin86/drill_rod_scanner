#!/usr/bin/env python3
"""软件调平：标定转盘旋转轴方向，自动写回安装配置文件。

原理：
  转盘轴不竖直时，绕世界 z 聚合的 3D 点云中，本应水平的平面（地面/桌面）
  会呈现倾斜。拟合该平面的法向量 n_fit，即可得到把点云校回水平的旋转
  R_align（把 n_fit 对齐到世界 z）。后续扫描聚合后整体应用 R_align，
  水平面即恢复水平（仿真验证：偏离 0.0000°）。

用法：
  # 方式 1：直接对已保存的扫描点云标定（离线，推荐先用这个验证）
  python scripts/level_scan.py --cloud output/scan_xxx/cloud.npy \
      --install-config configs/install_side_mount.yaml

  # 方式 2：连接硬件实时采一圈地面后标定（在线）
  python scripts/level_scan.py --install-config configs/install_side_mount.yaml \
      --port /dev/ttyUSB0 --start 500 --end 2500 --step 100

标定后会打印拟合结果并写回 config 文件（新增 turntable_level_correction）。

注意：拟合的是"水平面"——确保点云里含地面/桌面等大片水平平面，
且该平面未被其它物体大面积遮挡。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from install_config import InstallConfig  # noqa: E402


def fit_plane_normal(points: np.ndarray, voxel_size: float = 0.05) -> np.ndarray:
    """拟合点云中最大水平平面，返回其法向量（z>0）。

    points: (n,3) 点云。先按 voxel_size 体素降采样（百万级点云直接 SVD
    会内存爆炸），再最小二乘拟合平面，法向量取 SVD 最小奇异值方向。
    降采样不影响法向量方向（只影响精度），0.05m 对 1° 级倾斜标定足够。
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 10:
        raise ValueError(f"点云至少需要 10 个点，当前形状 {pts.shape}")

    if pts.shape[0] > 200_000:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = pcd.voxel_down_sample(voxel_size)
        pts = np.asarray(pcd.points)
        if pts.shape[0] < 10:
            raise ValueError(f"体素降采样后点数不足: {pts.shape[0]}（尝试增大 voxel_size）")

    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    return normal


def align_to_z_matrix(v: np.ndarray) -> np.ndarray:
    """把单位向量 v 旋转到世界 z=(0,0,1) 的旋转矩阵（最小旋转）。"""
    v = np.asarray(v, dtype=np.float64)
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(v, z)
    nrm = np.linalg.norm(axis)
    if nrm < 1e-12:
        return np.eye(3) if v[2] > 0 else np.diag([1.0, -1.0, -1.0])
    axis /= nrm
    theta = np.arccos(np.clip(np.dot(v, z), -1.0, 1.0))
    c, s = np.cos(theta), np.sin(theta)
    kx, ky, kz = axis
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def level_from_cloud(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """从点云标定：返回 (水平面法向量 n_fit, 校正矩阵 R_align)。

    R_align 应叠加在聚合后点云上：P_final = R_align @ P_aggregated。
    """
    n_fit = fit_plane_normal(points)
    r_align = align_to_z_matrix(n_fit)
    return n_fit, r_align


def write_level_correction(config_path: Path, r_align: np.ndarray) -> None:
    """把校正矩阵写入 install config YAML（turntable_level_correction）。"""
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    data["turntable_level_correction"] = [
        [float(x) for x in row] for row in np.asarray(r_align, dtype=np.float64)
    ]
    config_path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"[write] 校正矩阵已写入 {config_path}")


def apply_level_correction(points: np.ndarray, config: InstallConfig) -> np.ndarray:
    """对点云应用配置中的调平校正（若有）。"""
    if config.level_correction is not None:
        return points @ np.asarray(config.level_correction, dtype=np.float64).T
    return points


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--cloud", type=str, default="",
                     help="已保存的点云文件（.npy/.ply/.pcd），离线标定")
    src.add_argument("--scan", action="store_true",
                     help="连接硬件采一圈地面后标定（在线，用下方舵机参数）")
    parser.add_argument("--install-config", type=str, required=True,
                        help="安装配置 YAML 路径（标定结果写回此文件）")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="舵机串口（--scan 时）")
    parser.add_argument("--start", type=int, default=500, help="舵机起始位置（--scan 时）")
    parser.add_argument("--end", type=int, default=2500, help="舵机结束位置（--scan 时）")
    parser.add_argument("--step", type=int, default=100, help="位置步进（--scan 时）")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="每位置停留秒数（--scan 时）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只拟合打印，不写回配置文件")
    args = parser.parse_args()

    config_path = Path(args.install_config)
    if not config_path.exists():
        raise SystemExit(f"配置文件不存在: {config_path}")

    # 获取点云
    if args.cloud:
        cloud_path = Path(args.cloud)
        if cloud_path.suffix == ".npy":
            points = np.load(cloud_path)
        else:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(str(cloud_path))
            points = np.asarray(pcd.points)
        print(f"[load] 点云 {cloud_path} ({points.shape[0]} 点)")
    else:  # --scan：连接硬件采集
        from servo_sweep_scan import LakiBeamViewer  # noqa: PLC0415
        points = _scan_ground(args)
        print(f"[scan] 采集完成 ({points.shape[0]} 点)")

    n_fit, r_align = level_from_cloud(points)
    tilt = np.degrees(np.arccos(np.clip(n_fit[2], -1.0, 1.0)))
    print(f"[fit] 水平面法向量 = ({n_fit[0]:.4f}, {n_fit[1]:.4f}, {n_fit[2]:.4f})")
    print(f"[fit] 水平面倾斜角 = {tilt:.4f}°")
    print("[fit] 校正矩阵 R_align（聚合后叠加）:")
    for row in r_align:
        print(f"      [{row[0]:+.6f} {row[1]:+.6f} {row[2]:+.6f}]")

    if args.dry_run:
        print("[dry-run] 未写回配置文件")
        return
    write_level_correction(config_path, r_align)
    print("[done] 后续扫描请重新运行 servo_sweep_scan.py 使用同一配置文件")


def _scan_ground(args: argparse.Namespace) -> np.ndarray:
    """在线采集：驱动转盘转一圈，拼出 3D 点云（绕 z 聚合，未校正）。"""
    from servo_sweep_scan import (  # noqa: PLC0415
        LakiBeamViewer, mount_transform, scan_to_xy, servo_pos_to_angle, to_world,
    )
    import serial  # noqa: PLC0415
    import time  # noqa: PLC0415

    ser = serial.Serial(args.port, 115200, timeout=0.1)
    lidar = LakiBeamViewer(host_ip="0.0.0.0", port=2368)
    all_pts: list[np.ndarray] = []
    home = f"#000P{args.start:04d}T2000!"
    print(f"[scan] 归位: {home}")
    ser.write(home.encode())
    time.sleep(3.0)
    for pos in range(args.start, args.end + 1, args.step):
        cmd = f"#000P{pos:04d}T2000!"
        ser.write(cmd.encode())
        time.sleep(args.interval)
        scan = lidar.receive_scan()
        if not scan:
            continue
        frame = scan_to_xy(scan)
        frame = mount_transform(frame)
        frame = to_world(frame)
        angle = servo_pos_to_angle(pos)
        theta = np.deg2rad(angle)
        c, s = np.cos(theta), np.sin(theta)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        all_pts.append(frame @ rz.T)
        print(f"  pos={pos} angle={angle:.1f}° 帧 {frame.shape[0]} 点")
    ser.close()
    if not all_pts:
        raise SystemExit("未采集到任何点云")
    return np.vstack(all_pts)


if __name__ == "__main__":
    main()
