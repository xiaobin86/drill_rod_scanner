#!/usr/bin/env python3
"""将保存的点云发布为 ROS2 PointCloud2 topic，供 RViz 可视化。

读取 servo_sweep_scan.py --save-dir 保存的 cloud.npy（或 cloud.ply），
发布为 sensor_msgs/PointCloud2 到 /drill_scan_cloud topic。

用法（在 ROS2 环境，如 Docker pallet_vision:humble）:
  # 终端1: 发布点云（循环发布，RViz 可随时连接）
  python scripts/publish_pointcloud.py --file output/cloud.npy --topic /drill_scan_cloud

  # 终端2: RViz 添加 PointCloud2, 选择 /drill_scan_cloud 即可查看

依赖: 需在 ROS2 环境运行（rclpy, sensor_msgs），本机无 ROS2 时跳过。
"""

from __future__ import annotations

import argparse
import time

import numpy as np


def load_points(path: str) -> np.ndarray:
    """读取点云文件，返回 (n,3) numpy 数组。支持 .npy / .ply / .pcd。"""
    if path.endswith(".npy"):
        pts = np.load(path)
    else:
        import open3d as o3d

        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"点云形状应为 (n,3)，实际 {pts.shape}")
    return np.asarray(pts, dtype=np.float32)


def publish(points: np.ndarray, topic: str, frame_id: str, rate_hz: float) -> None:
    """创建 ROS2 节点并循环发布点云。"""
    import rclpy
    from rclpy.node import Node
    # ROS2 humble: point_cloud2 工具在 sensor_msgs_py 包（旧版在 sensor_msgs 里）
    import sensor_msgs_py.point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("drill_scan_publisher")
    pub = node.create_publisher(PointCloud2, topic, 10)
    header = Header(frame_id=frame_id)

    n = points.shape[0]
    print(f"发布 {n} 点到 topic {topic} (frame_id={frame_id}), 频率 {rate_hz}Hz")
    print("Ctrl+C 退出")

    try:
        while rclpy.ok():
            msg = pc2.create_cloud_xyz32(header, points)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(1.0 / rate_hz)
    except KeyboardInterrupt:
        print("\n用户中断, 退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--file", required=True, help="点云文件路径（.npy/.ply/.pcd）")
    parser.add_argument("--topic", default="/drill_scan_cloud", help="ROS2 topic 名")
    parser.add_argument("--frame-id", default="map", help="PointCloud2 frame_id")
    parser.add_argument("--rate", type=float, default=2.0, help="发布频率 Hz")
    args = parser.parse_args()

    points = load_points(args.file)
    publish(points, args.topic, args.frame_id, args.rate)


if __name__ == "__main__":
    main()
