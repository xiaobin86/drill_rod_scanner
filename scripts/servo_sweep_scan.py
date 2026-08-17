#!/usr/bin/env python3
"""舵机转盘扫描 + LakiBeam 点云拼接实时显示。

流程: 启动后先把转盘归位到 start 位置（等待到位），再从 start 逐步旋转到 end,
每个位置等雷达采一圈 2D 点云, 按转盘角度绕世界 z 轴（竖直）旋转后拼入累计点云,
Open3D 黑色背景绿色点实时显示。

坐标系约定:
  雷达系（安装方式）: x 向下, y 向左, z 向前; 自转扫描弧在 x-y 竖直平面（0° 指 +x 下）。
  世界系: z 轴竖直（转盘旋转轴）, x/y 水平（转盘平面）。
  坐标处理: 极坐标 -> 雷达系 xyz -> 安装配置变换(mount 棱镜相位 + to_world 安装姿态)
  -> 偏心校正(offset-y/z) -> 绕世界 z 轴（转盘轴）旋转聚合 3D 扫描面。
  安装方式可配置: 默认横装(configs/install_side_mount.yaml), 换安装方式用
  --install-config 指定 YAML（见 scripts/install_config.py 的格式说明）。
  因此拼接旋转轴默认 --axis z（= 世界转盘轴）。

用法:
  python scripts/servo_sweep_scan.py                          # 默认 500->1000, 步进10
  python scripts/servo_sweep_scan.py --port /dev/ttyUSB0 --start 100 --end 300 --step 20
  python scripts/servo_sweep_scan.py --continuous --total-time 60   # 连续转动模式
  python scripts/servo_sweep_scan.py --dry-run                # 不连串口, 只打印指令
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import serial
import threading

sys.path.insert(0, str(Path(__file__).resolve().parent))
from install_config import InstallConfig  # noqa: E402
from lakibeam_viewer import LakiBeamViewer, ScanPoint, scan_to_xy  # noqa: E402

# 安装配置（横装默认）。换安装方式：--install-config 指向 YAML，或改此默认。
_INSTALL: InstallConfig = InstallConfig.side_mount()


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


def rotation_matrix_axis_vector(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    """绕任意单位方向向量旋转 angle_deg 度的 3x3 矩阵（罗德里格斯公式）。

    用于转盘轴不竖直时：绕标定出的实际转盘轴（世界系向量）旋转聚合，
    而非假设轴 = 世界 z。axis 不必预先归一化（内部归一化）。
    """
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm == 0.0 or not np.all(np.isfinite(axis)):
        raise ValueError(f"无效旋转轴向量: {axis}")
    k = axis / norm
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    kx, ky, kz = k
    K = np.array([[0.0, -kz, ky],
                  [kz, 0.0, -kx],
                  [-ky, kx, 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def rotate_points(points: np.ndarray, axis: str | np.ndarray, angle_deg: float) -> np.ndarray:
    """将 (n,3) 点云绕指定轴旋转。axis 为字符串（x/y/z）或任意方向向量。"""
    if isinstance(axis, str):
        return points @ rotation_matrix(axis, angle_deg).T
    return points @ rotation_matrix_axis_vector(axis, angle_deg).T


def mount_transform(points: np.ndarray) -> np.ndarray:
    """棱镜 0° 参考相位：绕雷达 z 轴转 90°（由 _INSTALL 配置）。

    扫描点 xyz 是雷达系坐标（0° 指 +x 下、90° 指 +y 左），横装（绕世界 y
    向下转 90°，见 to_world）不改变它；但 LakiBeam 出厂 0° 参考与雷达 x 轴
    存在 90° 夹角（棱镜相位），安装对齐后 0° 应指 +y（左）。
    换安装方式时通过 --install-config 覆盖 _INSTALL。
    """
    return _INSTALL.mount_transform(points)


def to_world(points: np.ndarray) -> np.ndarray:
    """雷达系（安装后）→ 世界系（安装姿态，由 _INSTALL 配置）。

    横装 = 雷达绕世界 y 轴向下转 90°（出厂 x 前 → 横装 x 下）：
    世界 x=雷达 z（前）、世界 y=雷达 y（左）、世界 z=-雷达 x（下→上）。
    输入已含棱镜 90° 相位（mount_transform），0° 指 +y（左）、90° 指 -x（上）；
    变换后单帧弧 0° 指世界 y（水平左）、90° 指世界 z（竖直上），
    转盘绕世界 z 轴（转盘轴）旋转聚合 3D，旋转轴用 --axis z。
    """
    return _INSTALL.to_world(points)


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


# 舵机量程：位置 500-2500 固定映射 0-360 度
SERVO_POS_MIN = 500
SERVO_POS_MAX = 2500
SERVO_ANGLE_RANGE = 360.0

# 按高度着色：低→蓝，高→红（蓝-青-绿-黄-红 五段渐变，纯 numpy 实现）
HEIGHT_CMAP = np.array([
    [0.0, 0.0, 1.0],   # 蓝
    [0.0, 1.0, 1.0],   # 青
    [0.0, 1.0, 0.0],   # 绿
    [1.0, 1.0, 0.0],   # 黄
    [1.0, 0.0, 0.0],   # 红
])


def color_by_height(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """按点云 z（高度）值着色：z_min→蓝，z_max→红，返回 (n,3) RGB。"""
    if points.shape[0] == 0:
        return np.empty((0, 3))
    span = z_max - z_min
    if span <= 0.0:
        norm = np.zeros(points.shape[0])
    else:
        norm = np.clip((points[:, 2] - z_min) / span, 0.0, 1.0)
    # 五段线性插值
    scaled = norm * (len(HEIGHT_CMAP) - 1)
    idx = np.clip(scaled.astype(int), 0, len(HEIGHT_CMAP) - 2)
    frac = scaled - idx
    return HEIGHT_CMAP[idx] * (1 - frac[:, None]) + HEIGHT_CMAP[idx + 1] * frac[:, None]


def servo_pos_to_angle(pos: int) -> float:
    """舵机位置 P 值映射为旋转角度（度）。

    舵机量程固定：P=500 → 0°，P=2500 → 360°，线性映射。
    """
    frac = (pos - SERVO_POS_MIN) / (SERVO_POS_MAX - SERVO_POS_MIN)
    return SERVO_ANGLE_RANGE * frac


def pick_frame_index(timestamps: list[float], target_t: float) -> int:
    """从帧时间戳列表中找到最接近 target_t 的帧索引。

    转盘连续转动时每帧有接收时间戳；按位置步长抽帧时，
    每个位置对应一个时间点，取时间戳最接近该时间点的帧。
    """
    return min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - target_t))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="舵机串口")
    parser.add_argument("--install-config", type=str, default="",
                        help="雷达安装方式 YAML 配置路径（默认内置横装 side-mount，"
                             "见 configs/install_side_mount.yaml）")
    parser.add_argument("--baud", type=int, default=115200, help="舵机波特率")
    parser.add_argument("--start", type=int, default=500, help="舵机起始位置 P")
    parser.add_argument("--end", type=int, default=1000, help="舵机结束位置 P（含）")
    parser.add_argument("--step", type=int, default=10, help="每次位置增量")
    parser.add_argument("--interval", type=float, default=2.0, help="每个位置停留秒数")
    parser.add_argument("--continuous", action="store_true",
                        help="连续转动模式：发一条 start→end 命令连续转，全程记录帧后按步长抽帧融合")
    parser.add_argument("--total-time", type=float, default=60.0,
                        help="连续模式转盘总耗时（秒），仅 --continuous 时生效")
    parser.add_argument("--move-time", type=int, default=2000, help="舵机移动耗时 T（ms）")
    parser.add_argument("--home-wait", type=float, default=3.0,
                        help="归位后等待到位秒数（默认 3，含移动时间余量）")
    parser.add_argument("--servo-id", type=int, default=0, help="舵机 ID")
    parser.add_argument("--axis", default="z", choices=["x", "y", "z"],
                        help="绕世界系哪个轴旋转拼接（默认 z）。"
                             "软件调平后改由配置 turntable_axis_vector 指定实际转盘轴，此参数仅作回退")
    parser.add_argument("--lidar-port", type=int, default=2368, help="雷达数据端口")
    parser.add_argument("--offset-y", type=float, default=0.0,
                        help="光心相对转盘轴心的 y 偏移（米，雷达系 y 方向）")
    parser.add_argument("--offset-z", type=float, default=0.0,
                        help="光心相对转盘轴心的 z 偏移（米，雷达系 z 方向）")
    parser.add_argument("--max-range", type=float, default=50.0, help="最大显示距离（米）")
    parser.add_argument("--save-dir", type=str, default="",
                        help="扫描完成后保存点云到该目录（PLY+PCD+npz），留空不保存")
    parser.add_argument("--publish-topic", type=str, default="",
                        help="扫描完成后自动发布点云到该 ROS2 topic（如 /drill_scan_cloud），留空不发布")
    parser.add_argument("--publish-frame", type=str, default="map",
                        help="发布点云的 frame_id（默认 map）")
    parser.add_argument("--publish-rate", type=float, default=2.0,
                        help="发布频率 Hz")
    parser.add_argument("--grid", type=float, default=-1.0,
                        help="世界系水平面网格半宽（米），默认按 max-range 自动设置，0 关闭")
    parser.add_argument("--grid-step", type=float, default=1.0, help="网格线间距（米）")
    parser.add_argument("--dry-run", action="store_true", help="只打印舵机指令不连串口")
    parser.add_argument("--debug", action="store_true", help="打印每包诊断信息")
    args = parser.parse_args()

    # 安装配置：默认横装，可用 --install-config 指定 YAML 覆盖
    global _INSTALL
    if args.install_config:
        _INSTALL = InstallConfig.load(args.install_config)
        print(f"[install] 加载安装配置: {args.install_config}")
        print(f"          {_INSTALL.description}（棱镜相位 绕{_INSTALL.mount_axis} "
              f"{_INSTALL.mount_angle_deg}°，to_world x={_INSTALL.world_x} "
              f"y={_INSTALL.world_y} z={_INSTALL.world_z}）")
    else:
        print(f"[install] 默认安装配置: {_INSTALL.name}（{_INSTALL.description}）")

    # 未指定 --save-dir 时，默认用带时间戳目录（每次扫描独立，不覆盖）
    if not args.save_dir:
        args.save_dir = time.strftime("output/scan_%Y%m%d_%H%M%S")
        print(f"[dir] 默认保存目录: {args.save_dir}（可用 --save-dir 指定）")

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
    vis.get_render_option().point_size = 1.0  # 点渲染 1 像素

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
    points_per_frame = 4000  # 每圈点云上限（含余量，覆盖 10Hz 的 0.1° 分辨率 ~3600 点）
    max_points = len(positions) * points_per_frame
    cloud_buf = np.full((max_points, 3), np.nan, dtype=np.float64)
    color_buf = np.tile([0.0, 1.0, 0.0], (max_points, 1))  # 绿色点

    pcd: o3d.geometry.PointCloud | None = None
    total_points = 0

    def process_frame(scan, angle: float, label: str) -> None:
        """单帧雷达点：坐标变换 → 写入缓冲 → Open3D 更新。"""
        nonlocal total_points, pcd
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
        #    （x下/y左/z前 安装：偏移在雷达系 y/z 方向）
        frame[:, 1] += args.offset_y
        frame[:, 2] += args.offset_z
        # ③ 雷达系 → 世界系（z 竖直 = 转盘轴）
        frame = to_world(frame)
        # ④ 绕转盘轴旋转，聚合 3D 扫描面。
        #    轴 = 配置的实际转盘轴（默认世界 z；软件调平后为标定向量 n）
        rotated = rotate_points(frame, _INSTALL.rotation_axis_vector(), angle)
        # ⑤ 软件调平校正：转盘轴不竖直时，level_scan.py 拟合的水平面法向量
        #    → 校正矩阵（把水平面转回水平）叠加在聚合后点云上
        if _INSTALL.level_correction is not None:
            rotated = rotated @ _INSTALL.level_correction.T

        # 增量写入固定缓冲，避免 vstack 全量复制
        n = rotated.shape[0]
        cloud_buf[total_points:total_points + n] = rotated
        total_points += n
        print(f"  [scan] {label} angle={angle:.1f}° 帧 {n} 点, 累计 {total_points} 点")

        # 按高度着色：用当前有效点云的 z 范围给全部已有点重算颜色
        valid = cloud_buf[:total_points]
        color_buf[:total_points] = color_by_height(
            valid, valid[:, 2].min(), valid[:, 2].max()
        )

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
            pcd.colors = o3d.utility.Vector3dVector(color_buf)
            vis.update_geometry(pcd)
        vis.update_renderer()
        vis.poll_events()

    try:
        if args.continuous:
            # 连续转动模式（记录+抽帧）：
            # ① 一条命令从 start 连续转到 end，总时长 = --total-time
            # ② 过程中记录雷达每一帧（带接收时间戳）
            # ③ 转完后写文件，读回后按位置步长抽帧（取时间最近帧）→ 位置映射角度 → 融合
            total_s = max(args.total_time, 0.1)
            total_ms = int(total_s * 1000)
            cmd = f"#{args.servo_id:03d}P{args.end:04d}T{total_ms}!"
            print(f"[cont] 连续转动 {args.start}→{args.end}, 耗时 {total_s:.1f}s: {cmd}")
            if ser:
                ser.write(cmd.encode())
                ser.flush()

            # 记录全部帧
            t0 = time.monotonic()
            rec_ts: list[float] = []
            rec_frames: list[list[ScanPoint]] = []
            while True:
                elapsed = time.monotonic() - t0
                if elapsed >= total_s:
                    break
                scan = lidar.receive_scan()
                if scan is None or not scan:
                    print(f"  [rec] t={elapsed:.1f}s: 雷达无数据")
                    continue
                rec_ts.append(elapsed)
                rec_frames.append(scan)
                if len(rec_frames) % 50 == 0:
                    print(f"  [rec] 已记录 {len(rec_frames)} 帧, t={elapsed:.1f}s")
            print(f"[rec] 共记录 {len(rec_frames)} 帧, 总耗时 {total_s:.1f}s")

            # 空数据防护：一帧都没采到则明确报错，避免后续 min() 崩溃
            if not rec_frames:
                print("[rec] 错误: 未采到任何雷达帧，请检查雷达网络/配置后重试")
                raise RuntimeError("连续模式未采到任何雷达帧")

            # 写文件再读回
            rec_path = Path(args.save_dir) if args.save_dir else Path("output")
            rec_path.mkdir(parents=True, exist_ok=True)
            rec_file = rec_path / "frames.npz"
            np.savez(rec_file, ts=np.array(rec_ts),
                     frames=np.array(rec_frames, dtype=object))
            print(f"[rec] 帧数据已写入 {rec_file}")

            rec = np.load(rec_file, allow_pickle=True)
            rec_ts = list(rec["ts"])
            rec_frames = list(rec["frames"])

            # 抽帧间隔 = max(位置步长间隔, 雷达帧间隔)
            # 位置步长间隔：step 对应的运动时间 = step/(end-start) × total_s
            # 雷达帧间隔：记录时间戳的平均间隔
            # 当位置步长比雷达帧更密时，物理上无法获得更多独立帧，
            # 自动退化为按雷达帧间隔抽帧（每帧独立不重复）
            span = (args.end - args.start) if args.end > args.start else 1
            pos_interval = args.step / span * total_s
            frame_intervals = np.diff(rec_ts)
            frame_interval = float(np.mean(frame_intervals)) if len(frame_intervals) else total_s
            sample_interval = max(pos_interval, frame_interval)

            if pos_interval < frame_interval:
                print(f"[warn] 位置步长间隔 {pos_interval:.3f}s < 雷达帧间隔 {frame_interval:.3f}s，"
                      f"抽帧间隔自动取 {sample_interval:.3f}s（每帧独立）")

            # 按抽帧间隔生成时间点序列，取时间戳最近帧
            t = 0.0
            while t <= total_s:
                idx = pick_frame_index(rec_ts, t)
                scan = rec_frames[idx]
                # 角度按时间比例映射（匀速转动下 = 位置映射）
                frac = t / total_s if total_s > 0 else 0.0
                angle = frac * SERVO_ANGLE_RANGE
                process_frame(scan, angle, f"t={rec_ts[idx]:.1f}s")
                t += sample_interval
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

                angle = servo_pos_to_angle(pos)
                process_frame(scan, angle, f"pos={pos}")

        print(f"\n扫描完成: 累计 {total_points} 点")

        # 保存点云：取缓冲中有效部分（NaN 填充之外）
        if args.save_dir and total_points > 0:
            valid = cloud_buf[:total_points]
            saved = save_cloud(valid, args.save_dir, "ply")
            print(f"[save] 点云已保存到 {args.save_dir}")
            for kind, path in saved.items():
                print(f"  {kind}: {path}")

        # 自动发布到 ROS2 topic（后台线程，不阻塞 Open3D 窗口）
        publish_thread = None
        if args.publish_topic and total_points > 0:
            try:
                from publish_pointcloud import publish

                valid = cloud_buf[:total_points]
                publish_thread = threading.Thread(
                    target=publish,
                    args=(valid, args.publish_topic, args.publish_frame, args.publish_rate),
                    daemon=True,
                )
                publish_thread.start()
                print(f"[pub] 正在发布 {total_points} 点到 topic {args.publish_topic}"
                      f" (frame_id={args.publish_frame})，RViz 可直接查看")
            except Exception as exc:  # noqa: BLE001
                print(f"[pub] 自动发布失败（需要 ROS2 环境）: {exc}")

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
