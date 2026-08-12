#!/usr/bin/env python3
"""舵机转盘扫描 + LakiBeam 点云拼接实时显示。

流程: 舵机带动转盘从 start 位置逐步旋转到 end 位置, 每个位置等雷达采一圈 2D 点云,
按转盘角度绕世界 z 轴（竖直）旋转后拼入累计点云, Open3D 黑色背景绿色点实时显示。

坐标系约定:
  雷达横装: x 向前, y 朝上, z 向右; 自转扫描弧在 x-y 竖直平面。
  世界系: z 轴竖直（转盘旋转轴）, 与雷达系 y 轴同向。
  转盘绕世界 z 轴水平旋转, 把不同时刻的竖直扫描弧聚合成 3D 扫描面。
  因此拼接旋转轴默认 --axis y（= 世界 z）。

用法:
  python scripts/servo_sweep_scan.py                          # 默认 500->1000, 步进10
  python scripts/servo_sweep_scan.py --port /dev/ttyUSB0 --start 100 --end 300 --step 20
  python scripts/servo_sweep_scan.py --axis y --angle-start 0 --angle-end 180
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


def mount_transform(points: np.ndarray) -> np.ndarray:
    """出厂雷达系 → 横装后坐标（数学上恒等，保留步骤以便扩展）。

    物理：LakiBeam 旋转镜轴 = 雷达 z 轴。出厂 z 朝上、扫描平面 x-y 水平；
    横装后 z 轴（镜轴）朝右、扫描平面 x-y 竖直。
    由于扫描平面固定在雷达坐标系的 x-y 平面，坐标系跟随雷达旋转，
    扫描点坐标数值不变（x=r·cosθ, y=r·sinθ, z=0），仅 y 轴语义从
    "横向"变为"朝上"。若实际安装有额外倾斜，在此叠加旋转矩阵。
    """
    return points


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
    parser.add_argument("--axis", default="y", choices=["x", "y", "z"],
                        help="点云旋转轴（默认 y = 世界 z 竖直转盘轴）")
    parser.add_argument("--angle-start", type=float, default=0.0,
                        help="start 位置对应的旋转角度（度）")
    parser.add_argument("--angle-end", type=float, default=180.0,
                        help="end 位置对应的旋转角度（度）")
    parser.add_argument("--lidar-port", type=int, default=2368, help="雷达数据端口")
    parser.add_argument("--offset-x", type=float, default=0.0,
                        help="光心相对转盘轴心的 x 偏移（米，旋转前校正）")
    parser.add_argument("--offset-z", type=float, default=0.0,
                        help="光心相对转盘轴心的 z 偏移（米，旋转前校正）")
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

    # 预分配固定大小缓冲：点数永不变，避免每帧 remove/add 重建 GPU 缓冲导致的卡顿。
    # 未用部分以 NaN 填充，Open3D 渲染时跳过且不参与包围盒。
    positions = list(range(args.start, args.end + 1, args.step))
    points_per_frame = 2500  # 每圈点云上限（含余量）
    max_points = len(positions) * points_per_frame
    cloud_buf = np.full((max_points, 3), np.nan, dtype=np.float64)
    color_buf = np.tile([0.0, 1.0, 0.0], (max_points, 1))  # 绿色点

    pcd: o3d.geometry.PointCloud | None = None
    total_points = 0
    try:
        for pos in positions:
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

            # 三步坐标处理：
            # ① 极坐标 → 雷达系 xyz（x 前/y 上/z 右，扫描弧竖直）
            frame = scan_to_xy(scan)
            dist = np.linalg.norm(frame[:, :2], axis=1)
            frame = frame[dist <= args.max_range]
            if frame.shape[0] == 0:
                print(f"  [skip] 位置 {pos}: 滤波后无点")
                continue

            angle = servo_pos_to_angle(pos, args.start, args.end,
                                       args.angle_start, args.angle_end)
            # ② 横装变换（出厂系 → 横装系，数学上恒等）
            frame = mount_transform(frame)
            # ③ 光心偏心校正：平移到转盘轴心，再绕转盘轴旋转
            frame[:, 0] -= args.offset_x
            frame[:, 2] -= args.offset_z
            rotated = rotate_points(frame, args.axis, angle)

            # 增量写入固定缓冲，避免 vstack 全量复制
            n = rotated.shape[0]
            cloud_buf[total_points:total_points + n] = rotated
            total_points += n
            print(f"  [scan] pos={pos} angle={angle:.1f}° 帧 {n} 点, 累计 {total_points} 点")

            # 首帧真实数据写入后才 add_geometry：此时包围盒含有效点。
            # （全 NaN 或空点云 add 会得到退化包围盒导致不渲染。）
            if pcd is None:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(cloud_buf)
                pcd.colors = o3d.utility.Vector3dVector(color_buf)
                vis.add_geometry(pcd)
                ctr = vis.get_view_control()
                ctr.set_front([0.0, 0.0, 1.0])
                ctr.set_up([0.0, 1.0, 0.0])
                ctr.set_lookat([0.0, 0.0, 0.0])
            else:
                # 点数不变，update_geometry 走快速路径
                pcd.points = o3d.utility.Vector3dVector(cloud_buf)
                vis.update_geometry(pcd)
            vis.update_renderer()
            vis.poll_events()

        print(f"\n扫描完成: {len(positions)} 个位置, 累计 {total_points} 点")
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
