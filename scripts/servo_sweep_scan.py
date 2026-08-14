#!/usr/bin/env python3
"""舵机转盘扫描 + LakiBeam 点云拼接实时显示。

流程: 启动后先把转盘归位到 start 位置（等待到位），再从 start 逐步旋转到 end,
每个位置等雷达采一圈 2D 点云, 按转盘角度绕世界 z 轴（竖直）旋转后拼入累计点云,
Open3D 黑色背景绿色点实时显示。

坐标系约定:
  雷达系（实测安装）: x 向前, y 朝上, z 向右; 自转扫描弧在 x-y 竖直平面（0° 指前）。
  世界系: z 轴竖直（转盘旋转轴）, x/y 水平（转盘平面）。
  坐标处理: 极坐标 -> 雷达系 xyz -> 横装变换(恒等) -> 偏心校正(offset-x/z)
  -> to_world(雷达系->世界系) -> 绕世界 z 轴（转盘轴）旋转聚合 3D 扫描面。
  因此拼接旋转轴默认 --axis z（= 世界转盘轴）。

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


def mount_transform(points: np.ndarray) -> np.ndarray:
    """出厂雷达系 → 横装后坐标（数学上恒等，保留步骤以便扩展）。

    物理：LakiBeam 旋转镜轴 = 雷达 z 轴。出厂 z 朝上、扫描平面 x-y 水平；
    横装后 z 轴（镜轴）朝右、扫描平面 x-y 竖直。
    由于扫描平面固定在雷达坐标系的 x-y 平面，坐标系跟随雷达旋转，
    扫描点坐标数值不变（x=r·cosθ, y=r·sinθ, z=0），仅 y 轴语义从
    "横向"变为"朝上"。若实际安装有额外倾斜，在此叠加旋转矩阵。
    """
    return points


def to_world(points: np.ndarray) -> np.ndarray:
    """雷达系 → 世界系（转盘系）。

    雷达系（实测安装）：x 前、y 上、z 右，扫描弧在 x-y 竖直面（0° 指前）。
    世界系：z 竖直（转盘旋转轴）、x/y 水平。
    映射：世界 x=雷达 x（前）、世界 y=雷达 z（右）、世界 z=雷达 y（上）。
    点已在世界系，转盘绕世界 z 轴旋转，拼接旋转轴用 --axis z。
    """
    return points[:, [0, 2, 1]]  # (x, y, z) -> (x_radar, z_radar, y_radar)


def save_cloud(points: np.ndarray, output_dir: str, cloud_format: str = "ply") -> dict[str, Path]:
    """将 (n,3) 点云保存为 PLY/PCD 文件 + numpy 原始数据。"""
    import open3d as o3d

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if cloud_format == "pcd":
        cloud_path = out / "cloud.pcd"
        o3d.io.write_point_cloud(str(cloud_path), pcd, write_ascii=True)
    else:
        cloud_path = out / "cloud.ply"
        o3d.io.write_point_cloud(str(cloud_path), pcd)

    np_path = out / "cloud.npy"
    np.save(np_path, points)

    return {"cloud": cloud_path, "numpy": np_path}


def create_ground_grid(
    half_extent: float, step: float, z_level: float = 0.0
) -> "o3d.geometry.LineSet":
    """在世界系 z=z_level 水平面生成 x-y 网格线（静态背景）。

    网格范围 [-half_extent, half_extent] × [-half_extent, half_extent]，
    线间距 step。用于可视化时对照世界系水平面。
    """
    import open3d as o3d

    ticks = np.arange(-half_extent, half_extent + step, step)
    points: list[np.ndarray] = []
    lines: list[tuple[int, int]] = []

    # 平行 x 轴的线（固定 y，扫 x）
    for y in ticks:
        base = len(points)
        points.append(np.array([-half_extent, y, z_level]))
        points.append(np.array([half_extent, y, z_level]))
        lines.append((base, base + 1))
    # 平行 y 轴的线（固定 x，扫 y）
    for x in ticks:
        base = len(points)
        points.append(np.array([x, -half_extent, z_level]))
        points.append(np.array([x, half_extent, z_level]))
        lines.append((base, base + 1))

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.asarray(points))
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines))
    line_set.paint_uniform_color([0.35, 0.35, 0.35])  # 灰色网格线
    return line_set


def create_world_axes(length: float = 1.0) -> "o3d.geometry.LineSet":
    """世界系原点三轴：X 红、Y 绿、Z 蓝，用于标定点云方位。"""
    import open3d as o3d

    pts = np.array([[0, 0, 0], [length, 0, 0],
                    [0, 0, 0], [0, length, 0],
                    [0, 0, 0], [0, 0, length]])
    lines = np.array([[0, 1], [2, 3], [4, 5]])
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def servo_pos_to_angle(
    pos: int, start: int, end: int, angle_start: float, angle_end: float
) -> float:
    """舵机位置 P 值线性映射为旋转角度（度）。"""
    if end <= start:
        return angle_start
    frac = (pos - start) / (end - start)
    return angle_start + frac * (angle_end - angle_start)


def theta_at_time(
    elapsed_s: float, total_s: float, angle_start: float, angle_end: float
) -> float:
    """连续转动模式：按已过时间线性推算当前转盘角度（度）。

    转盘从 angle_start 连续转到 angle_end，耗时 total_s；
    elapsed_s 时刻的角度按线性插值，超时后钳位在 angle_end。
    """
    if total_s <= 0.0:
        return angle_start
    frac = min(max(elapsed_s / total_s, 0.0), 1.0)
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
    parser.add_argument("--continuous", action="store_true",
                        help="连续转动模式：发一条 start→end 命令连续转，每 interval 采帧按时间算角度")
    parser.add_argument("--move-time", type=int, default=2000, help="舵机移动耗时 T（ms）")
    parser.add_argument("--home-wait", type=float, default=3.0,
                        help="归位后等待到位秒数（默认 3，含移动时间余量）")
    parser.add_argument("--servo-id", type=int, default=0, help="舵机 ID")
    parser.add_argument("--axis", default="z", choices=["x", "y", "z"],
                        help="绕世界系哪个轴旋转拼接（默认 z = 转盘竖直轴）")
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
    parser.add_argument("--save-dir", type=str, default="",
                        help="扫描完成后保存点云到该目录（PLY+PCD+npz），留空不保存")
    parser.add_argument("--grid", type=float, default=-1.0,
                        help="世界系水平面网格半宽（米），默认按 max-range 自动设置，0 关闭")
    parser.add_argument("--grid-step", type=float, default=1.0, help="网格线间距（米）")
    parser.add_argument("--dry-run", action="store_true", help="只打印舵机指令不连串口")
    parser.add_argument("--debug", action="store_true", help="打印每包诊断信息")
    args = parser.parse_args()

    # 1. 舵机串口
    ser = None
    if not args.dry_run:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"[open] 舵机 {args.port} @ {args.baud} baud")

    # 1.5 归位初始化：先让转盘转到起始位置，等到位后再开始采集
    home_cmd = f"#{args.servo_id:03d}P{args.start:04d}T{args.move_time}!"
    print(f"[home] 转盘归位到起始位置 {args.start}: {home_cmd}")
    if ser:
        ser.write(home_cmd.encode())
        ser.flush()
    print(f"[home] 等待 {args.home_wait}s 到位...")
    time.sleep(args.home_wait)

    if args.dry_run:
        print("\n[dry-run] 仅验证舵机指令序列（含归位），不连接雷达/Open3D，退出")
        return

    # 2. 雷达 UDP
    lidar = LakiBeamViewer(host_ip="0.0.0.0", port=args.lidar_port, debug=args.debug)
    lidar.connect()

    # 3. Open3D 可视化
    import open3d as o3d

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Servo Sweep Scan", width=900, height=700)
    vis.get_render_option().background_color = np.array([0.0, 0.0, 0.0])  # 黑色背景

    # 世界系 z=0 水平面网格（静态背景，画一次即可）
    grid = None
    grid_half = args.grid if args.grid >= 0.0 else args.max_range
    if grid_half > 0.0:
        grid = create_ground_grid(grid_half, args.grid_step, z_level=0.0)
        vis.add_geometry(grid)
        # 渲染器首轮 poll 前 add 的几何体不会显示，需先驱动一帧
        vis.poll_events()
        vis.update_renderer()

    # 预分配固定大小缓冲：点数永不变，避免每帧 remove/add 重建 GPU 缓冲导致的卡顿。
    # 未用部分以 NaN 填充，Open3D 渲染时跳过且不参与包围盒。
    positions = list(range(args.start, args.end + 1, args.step))
    points_per_frame = 2500  # 每圈点云上限（含余量）
    max_points = len(positions) * points_per_frame
    cloud_buf = np.full((max_points, 3), np.nan, dtype=np.float64)
    color_buf = np.tile([0.0, 1.0, 0.0], (max_points, 1))  # 绿色点

    pcd: o3d.geometry.PointCloud | None = None
    total_points = 0

    def process_frame(scan, angle: float, label: str) -> None:
        """单帧雷达点：坐标变换 → 写入缓冲 → Open3D 更新。"""
        nonlocal total_points
        frame = scan_to_xy(scan)
        dist = np.linalg.norm(frame[:, :2], axis=1)
        frame = frame[dist <= args.max_range]
        if frame.shape[0] == 0:
            print(f"  [skip] {label}: 滤波后无点")
            return

        # ① 横装变换（出厂系 → 横装系，数学上恒等）
        frame = mount_transform(frame)
        # ② 光心偏心校正：光心绕转盘轴做圆弧运动，
        #    世界点 = Rz(θ)·(雷达系测量 + 光心偏移d)，须先加 d 再旋转
        frame[:, 0] += args.offset_x
        frame[:, 2] += args.offset_z
        # ③ 雷达系 → 世界系（z 竖直 = 转盘轴）
        frame = to_world(frame)
        # ④ 绕世界 z 轴（转盘轴）旋转，聚合 3D 扫描面
        rotated = rotate_points(frame, args.axis, angle)

        # 增量写入固定缓冲，避免 vstack 全量复制
        n = rotated.shape[0]
        cloud_buf[total_points:total_points + n] = rotated
        total_points += n
        print(f"  [scan] {label} angle={angle:.1f}° 帧 {n} 点, 累计 {total_points} 点")

        # 首帧真实数据写入后才 add_geometry：此时包围盒含有效点。
        # （全 NaN 或空点云 add 会得到退化包围盒导致不渲染。）
        if pcd is None:
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

    try:
        if args.continuous:
            # 连续转动模式：一条命令从 start 转到 end，耗时 = 步进数 × interval
            n_steps = max((args.end - args.start) // args.step, 1)
            total_s = n_steps * args.interval
            total_ms = int(total_s * 1000)
            cmd = f"#{args.servo_id:03d}P{args.end:04d}T{total_ms}!"
            print(f"[cont] 连续转动 {args.start}→{args.end}, 耗时 {total_s:.1f}s: {cmd}")
            if ser:
                ser.write(cmd.encode())
                ser.flush()
            t0 = time.monotonic()
            frame_idx = 0
            while True:
                elapsed = time.monotonic() - t0
                if elapsed >= total_s:
                    break
                scan = lidar.receive_scan()
                if scan is None or not scan:
                    print(f"  [skip] t={elapsed:.1f}s: 雷达无数据")
                    continue
                angle = theta_at_time(elapsed, total_s,
                                      args.angle_start, args.angle_end)
                process_frame(scan, angle, f"t={elapsed:.1f}s")
                frame_idx += 1
                # 等到下一个 interval 时间点再采下一帧
                next_t = t0 + frame_idx * args.interval
                while time.monotonic() < next_t and time.monotonic() - t0 < total_s:
                    time.sleep(0.01)
        else:
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

                angle = servo_pos_to_angle(pos, args.start, args.end,
                                           args.angle_start, args.angle_end)
                process_frame(scan, angle, f"pos={pos}")

        print(f"\n扫描完成: 累计 {total_points} 点")

        # 保存点云：取缓冲中有效部分（NaN 填充之外）
        if args.save_dir and total_points > 0:
            valid = cloud_buf[:total_points]
            saved = save_cloud(valid, args.save_dir, "ply")
            print(f"[save] 点云已保存到 {args.save_dir}")
            for kind, path in saved.items():
                print(f"  {kind}: {path}")

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
