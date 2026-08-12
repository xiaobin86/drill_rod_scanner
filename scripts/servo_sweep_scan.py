#!/usr/bin/env python3
"""舵机旋转扫描 + LakiBeam 点云拼接实时显示。

流程: 舵机从 start 位置逐步旋转到 end 位置, 每个位置等雷达采一圈 2D 点云,
按舵机位置映射的角度绕旋转轴旋转后拼入累计点云, Open3D 黑色背景绿色点实时显示。

坐标系约定:
  雷达系 (横装): x 向前, y 朝天, z 向右, 扫描平面为 x-z 水平面 (y 固定为扫描高度)。
  世界系: 与雷达初始位姿对齐。舵机绕世界坐标 z 轴旋转雷达 (水平横向轴),
  扫描平面随之翻转, 扫过不同高度, 拼接得到 3D 点云。因此默认 --axis z。

用法:
  python scripts/servo_sweep_scan.py                          # 默认 500->1000, 步进10
  python scripts/servo_sweep_scan.py --port /dev/ttyUSB0 --start 100 --end 300 --step 20
  python scripts/servo_sweep_scan.py --axis z --angle-start 0 --angle-end 180
  python scripts/servo_sweep_scan.py --dry-run                # 不连串口, 只打印指令
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lakibeam_viewer import LakiBeamViewer, scan_to_xy  # noqa: E402


def rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """绕指定轴旋转 angle_deg 度的 3x3 矩阵（右手系）。"""
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0],
                         [0.0, c, -s],
                         [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s],
                         [0.0, 1.0, 0.0],
                         [-s, 0.0, c]])
    if axis == "z":
        return np.array([[c, -s, 0.0],
                         [s, c, 0.0],
                         [0.0, 0.0, 1.0]])
    raise ValueError(f"不支持的旋转轴: {axis}（可选 x/y/z）")


def rotate_points(points: np.ndarray, axis: str, angle_deg: float) -> np.ndarray:
    """将 (n,3) 点云绕指定轴旋转。"""
    return points @ rotation_matrix(axis, angle_deg).T


def servo_pos_to_angle(
    pos: int, start: int, end: int, angle_start: float, angle_end: float
) -> float:
    """舵机位置 P 值线性映射为旋转角度（度）。"""
    if end <= start:
        return angle_start
    frac = (pos - start) / (end - start)
    return angle_start + frac * (angle_end - angle_start)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="舵机串口")
    parser.add_argument("--baud", type=int, default=115200, help="舵机波特率")
    parser.add_argument("--start", type=int, default=500, help="舵机起始位置 P")
    parser.add_argument("--end", type=int, default=1000, help="舵机结束位置 P（含）")
    parser.add_argument("--step", type=int, default=10, help="每次位置增量")
    parser.add_argument("--interval", type=float, default=2.0, help="每个位置停留秒数")
    parser.add_argument("--move-time", type=int, default=2000, help="舵机移动耗时 T（ms）")
    parser.add_argument("--servo-id", type=int, default=0, help="舵机 ID")
    parser.add_argument("--axis", default="z", choices=["x", "y", "z"],
                        help="点云旋转轴（默认 z，取决于舵机安装方向）")
    parser.add_argument("--angle-start", type=float, default=0.0,
                        help="start 位置对应的旋转角度（度）")
    parser.add_argument("--angle-end", type=float, default=180.0,
                        help="end 位置对应的旋转角度（度）")
    parser.add_argument("--lidar-port", type=int, default=2368, help="雷达数据端口")
    parser.add_argument("--height", type=float, default=0.0, help="雷达扫描高度（米）")
    parser.add_argument("--max-range", type=float, default=50.0, help="最大显示距离（米）")
    parser.add_argument("--dry-run", action="store_true", help="只打印舵机指令不连串口")
    parser.add_argument("--debug", action="store_true", help="打印每包诊断信息")
    args = parser.parse_args()

    # 1. 舵机串口
    ser = None
    if not args.dry_run:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"[open] 舵机 {args.port} @ {args.baud} baud")

    # 2. 雷达 UDP
    lidar = LakiBeamViewer(host_ip="0.0.0.0", port=args.lidar_port, debug=args.debug)
    lidar.connect()

    # 3. Open3D 可视化
    import open3d as o3d

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Servo Sweep Scan", width=900, height=700)
    vis.get_render_option().background_color = np.array([0.0, 0.0, 0.0])  # 黑色背景
    pcd = o3d.geometry.PointCloud()
    geometry_added = False
    accumulated: list[np.ndarray] = []

    try:
        for pos in range(args.start, args.end + 1, args.step):
            cmd = f"#{args.servo_id:03d}P{pos:04d}T{args.move_time}!"
            print(f"[servo] {cmd}")
            if ser:
                ser.write(cmd.encode())
                ser.flush()
            time.sleep(args.interval)

            scan = lidar.receive_scan()
            if scan is None or not scan:
                print(f"  [skip] 位置 {pos}: 雷达无数据")
                continue

            frame = scan_to_xy(scan, height_m=args.height)
            dist = np.linalg.norm(frame[:, :2], axis=1)
            frame = frame[dist <= args.max_range]

            angle = servo_pos_to_angle(pos, args.start, args.end,
                                       args.angle_start, args.angle_end)
            rotated = rotate_points(frame, args.axis, angle)
            accumulated.append(rotated)
            cloud = np.vstack(accumulated)
            print(f"  [scan] pos={pos} angle={angle:.1f}° 帧 {len(rotated)} 点, 累计 {len(cloud)} 点")

            pcd.points = o3d.utility.Vector3dVector(cloud)
            pcd.colors = o3d.utility.Vector3dVector(
                np.tile([0.0, 1.0, 0.0], (len(cloud), 1))  # 绿色点
            )
            if not geometry_added:
                vis.add_geometry(pcd)
                ctr = vis.get_view_control()
                ctr.set_front([0.0, 0.0, 1.0])
                ctr.set_up([0.0, 1.0, 0.0])
                ctr.set_lookat([0.0, 0.0, 0.0])
                geometry_added = True
            else:
                vis.update_geometry(pcd)
            vis.update_renderer()

        print(f"\n扫描完成: {len(accumulated)} 帧, 累计 {len(cloud) if accumulated else 0} 点")
        print("窗口保持打开, 可鼠标旋转/缩放查看, Ctrl+C 或关窗退出")
        while vis.poll_events():
            vis.update_renderer()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n用户中断, 退出")
    finally:
        if ser:
            ser.close()
        lidar.close()
        vis.destroy_window()


if __name__ == "__main__":
    main()
