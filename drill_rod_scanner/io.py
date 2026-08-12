"""点云结果导出：PLY/PCD 文件 + 原始帧 numpy 文件。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from drill_rod_scanner.scanner import ScanResult


def save_result(result: ScanResult, output_dir: str | Path, cloud_format: str = "ply") -> dict[str, Path]:
    """将扫描结果导出到 output_dir，返回生成的文件路径字典。"""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    files: dict[str, Path] = {}

    # 原始帧与角度
    frames_path = out / "frames.npz"
    np.savez(
        frames_path,
        angles_deg=np.array(result.angles_deg),
        **{f"frame_{i}": f for i, f in enumerate(result.frames)},
    )
    files["frames"] = frames_path

    # 拼接点云
    if result.cloud is not None:
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(result.cloud)
        if cloud_format == "pcd":
            cloud_path = out / "cloud.pcd"
            o3d.io.write_point_cloud(str(cloud_path), pcd, write_ascii=True)
        else:
            cloud_path = out / "cloud.ply"
            o3d.io.write_point_cloud(str(cloud_path), pcd)
        files["cloud"] = cloud_path

    return files
