#!/usr/bin/env python3
"""LakiBeam LiDAR 网络接收 + Open3D 实时可视化脚本。

连接 LakiBeam 系列（LakiBeam 1/1S/1L）雷达，接收 MSOP UDP 数据包，
解析出距离/角度/回波强度，转换为直角坐标点云并用 Open3D 实时显示。

用法:
    # 0. 网线连接雷达，本机网卡配 192.168.198.1/24（雷达出厂 192.168.198.2）
    #    sudo ip addr add 192.168.198.1/24 dev <网卡名>
    # 1. 浏览器 http://192.168.198.2 确认 laser_enable=True, DataPort=2368
    conda run -n drill_rod_scanner python scripts/lakibeam_viewer.py
    conda run -n drill_rod_scanner python scripts/lakibeam_viewer.py --lidar-ip 192.168.198.2 --port 2368

依赖:
    numpy, open3d（项目 drill_rod_scanner 环境已包含）

协议参考: RichbeamTechnology/Lakibeam_ROS2_Driver (data_type.h / lakibeam1_scan.cpp)
MSOP 包结构 (1248 字节, 小端序):
    42B  UDP/IP 头（本脚本绑定本机端口直接 recv，socket 层已剥离）
    12 × Data Block (100B):
        2B DataFlag (0xEEFF)
        2B Azimuth      (单位 0.01°, uint16)
        16 × MeasuringResult (6B):
            2B Dist_1 + 1B RSSI_1   <- 最强回波
            2B Dist_2 + 1B RSSI_2   <- 最后回波（脚本忽略）
    4B  Timestamp (us)
    2B  Factory
块内 16 点角度线性插值: angle = Azimuth[j] + (Azimuth[j+1]-Azimuth[j])/16 * i
无效数据: DataFlag != 0xEEFF 或 dist == 0 时跳过
"""

from __future__ import annotations

import argparse
import socket
import struct
from dataclasses import dataclass

import numpy as np

# ---- MSOP 协议常量 ----
MSOP_DATA_BLOCKS = 12
MSOP_POINTS_PER_BLOCK = 16
MSOP_BLOCK_SIZE = 100  # 2 flag + 2 azimuth + 16*6
# UDP 载荷 = 12×100B Data Block + 4B Timestamp + 2B Factory = 1206 字节。
# （42B 网络头由内核剥离，不进入应用层载荷；tcpdump 实测 length 1206。）
MSOP_PACKET_SIZE = MSOP_DATA_BLOCKS * MSOP_BLOCK_SIZE + 4 + 2
DATA_FLAG = 0xEEFF
INVALID_DIST = 0


@dataclass
class ScanPoint:
    """一圈扫描中的一个有效测距点。"""

    angle: float  # 度 (0.01° 原始值 / 100)
    dist_mm: int
    rssi: int


class MSOPParser:
    """解析 LakiBeam MSOP UDP 数据包。"""

    @staticmethod
    def parse_packet(packet: bytes) -> list[ScanPoint]:
        """解析一个 MSOP 包，返回其中的有效测距点。

        与官方 ROS2 驱动一致：
        - 块内角度线性插值（相邻块 Azimuth 差 / 16）
        - 仅保留 DataFlag == 0xEEFF 且 dist != 0 的点
        """
        if len(packet) < MSOP_PACKET_SIZE:
            return []

        points: list[ScanPoint] = []
        offset = 0  # UDP 载荷直接从 DataFlag 开始，无网络头

        # 解析所有块的 Azimuth，先取前两块角度差计算块内插值分辨率。
        # （与官方 ROS2 驱动一致：resolution = (Azimuth[1] - Azimuth[0]) / 16，
        #  应用于包内所有块，包括第 0 块。）
        azimuths: list[float] = []
        flags: list[int] = []
        for block_idx in range(MSOP_DATA_BLOCKS):
            block_start = offset + block_idx * MSOP_BLOCK_SIZE
            flag = struct.unpack_from("<H", packet, block_start)[0]
            azimuth_raw = struct.unpack_from("<H", packet, block_start + 2)[0]
            flags.append(flag)
            azimuths.append(azimuth_raw / 100.0)

        # 块内角度分辨率：相邻块角度差 / 16（差为正才有效）
        resolution = 0.0
        if len(azimuths) >= 2:
            diff = azimuths[1] - azimuths[0]
            if diff > 0:
                resolution = diff / MSOP_POINTS_PER_BLOCK

        for block_idx in range(MSOP_DATA_BLOCKS):
            flag = flags[block_idx]
            azimuth = azimuths[block_idx]
            block_start = offset + block_idx * MSOP_BLOCK_SIZE

            if flag != DATA_FLAG:
                continue

            for i in range(MSOP_POINTS_PER_BLOCK):
                # 每个测距结果 6 字节：Dist_1(2B) RSSI_1(1B) Dist_2(2B) RSSI_2(1B)
                result_start = block_start + 4 + i * 6
                dist_1, rssi_1 = struct.unpack_from(
                    "<HB", packet, result_start
                )
                if dist_1 == INVALID_DIST:
                    continue  # 无回波

                angle = azimuth + resolution * i
                points.append(ScanPoint(angle=angle, dist_mm=dist_1, rssi=rssi_1))

        return points


def scan_to_xy(points: list[ScanPoint], z: float = 0.0) -> np.ndarray:
    """将一圈测距点转换为 (n,3) 直角坐标点云。

    LakiBeam 是水平扫描的 2D 雷达：x = r*cos(θ), y = r*sin(θ), z 固定。
    距离单位 mm -> m。角度按雷达手册定义（0° 指向 x 正方向，逆时针）。
    """
    if not points:
        return np.empty((0, 3))
    angles = np.array([p.angle for p in points], dtype=np.float64)
    dists = np.array([p.dist_mm for p in points], dtype=np.float64) / 1000.0
    thetas = np.deg2rad(angles)
    xy = np.column_stack([
        dists * np.cos(thetas),
        dists * np.sin(thetas),
        np.full(len(points), z, dtype=np.float64),
    ])
    return xy


class LakiBeamViewer:
    """接收 LakiBeam UDP 数据流并用 Open3D 实时可视化。"""

    def __init__(
        self,
        host_ip: str = "0.0.0.0",
        port: int = 2368,
        z: float = 0.0,
        min_rssi: int = 0,
        max_range_m: float = 50.0,
    ) -> None:
        self.host_ip = host_ip
        self.port = port
        self.z = z
        self.min_rssi = min_rssi
        self.max_range_m = max_range_m
        self.sock: socket.socket | None = None

    def connect(self) -> None:
        """绑定本机 UDP 端口，等待雷达数据。"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host_ip, self.port))
        self.sock.settimeout(1.0)  # 1 秒超时，用于主循环退出检查
        print(f"已绑定 UDP {self.host_ip}:{self.port}，等待 LakiBeam 数据...")
        print("提示: 确保雷达已启动测距（Web 配置页 laser_enable=True），且向本机端口发送数据")

    def receive_scan(self) -> list[ScanPoint] | None:
        """接收数据直到集齐一圈（Azimuth 重新回到起点即为一圈）。

        返回一圈的有效点列表；超时返回 None。
        """
        if self.sock is None:
            raise RuntimeError("未连接，请先调用 connect()")

        all_points: list[ScanPoint] = []
        seen_azimuth_zero = False
        while True:
            try:
                packet, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                return None

            points = MSOPParser.parse_packet(packet)
            all_points.extend(points)

            # 一圈结束标志：包内 Azimuth 回到起点（0° 附近）
            if points and points[0].angle < 1.0:
                if seen_azimuth_zero:
                    break
                seen_azimuth_zero = True

        return all_points

    def visualize(self, fps: int = 20) -> None:
        """Open3D 实时可视化循环，Ctrl+C 或关窗退出。"""
        import open3d as o3d

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="LakiBeam Live", width=800, height=600)
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)

        # 固定视角：从上方俯视扫描平面
        ctr = vis.get_view_control()
        ctr.set_front([0.0, 0.0, 1.0])
        ctr.set_up([0.0, 1.0, 0.0])

        frame_count = 0
        try:
            while vis.poll_events():
                scan = self.receive_scan()
                if scan is None:
                    continue  # 超时无数据，继续等

                # 滤波：RSSI 下限 + 最大距离
                pts = scan_to_xy(scan, z=self.z)
                if pts.shape[0] == 0:
                    continue
                dist = np.linalg.norm(pts[:, :2], axis=1)
                mask = dist <= self.max_range_m
                pts = pts[mask]

                pcd.points = o3d.utility.Vector3dVector(pts)
                vis.update_geometry(pcd)
                vis.update_renderer()

                frame_count += 1
                if frame_count % fps == 0:
                    print(f"帧 {frame_count}: {pts.shape[0]} 点")
        except KeyboardInterrupt:
            print("\n用户中断，退出")
        finally:
            vis.destroy_window()
            self.close()

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None


def main() -> None:
    parser = argparse.ArgumentParser(description="LakiBeam 雷达实时点云可视化")
    parser.add_argument("--lidar-ip", default="192.168.198.2",
                        help="雷达 IP（默认 192.168.198.2）")
    parser.add_argument("--port", type=int, default=2368,
                        help="接收数据端口（默认 2368，需与雷达配置一致）")
    parser.add_argument("--host-ip", default="0.0.0.0",
                        help="绑定本机 IP（默认 0.0.0.0 监听所有网卡）")
    parser.add_argument("--z", type=float, default=0.0,
                        help="2D 扫描平面的 z 高度（米）")
    parser.add_argument("--min-rssi", type=int, default=0,
                        help="回波强度下限过滤（默认 0 不过滤）")
    parser.add_argument("--max-range", type=float, default=50.0,
                        help="最大显示距离（米）")
    parser.add_argument("--fps", type=int, default=20,
                        help="打印帧信息的频率")
    args = parser.parse_args()

    print(f"目标雷达: {args.lidar_ip}")
    viewer = LakiBeamViewer(
        host_ip=args.host_ip,
        port=args.port,
        z=args.z,
        min_rssi=args.min_rssi,
        max_range_m=args.max_range,
    )
    viewer.connect()
    viewer.visualize(fps=args.fps)


if __name__ == "__main__":
    main()
