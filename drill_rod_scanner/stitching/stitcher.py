"""点云拼接：将各角度采集的雷达帧按舵机角度绕 Z 轴旋转变换后合并。"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def _rotation_z(angle_deg: float) -> np.ndarray:
    """绕 Z 轴旋转角度（度）的 3x3 旋转矩阵。"""
    theta = np.deg2rad(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """体素下采样：按 voxel 网格取每格质心，纯 numpy 实现（不依赖 open3d）。"""
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points
    voxel = np.floor(points / voxel_size).astype(np.int64)
    # 用字典按体素坐标分组并求质心
    groups: dict[tuple[int, int, int], list[np.ndarray]] = {}
    for key, pt in zip(map(tuple, voxel), points):
        groups.setdefault(key, []).append(pt)
    return np.array([np.mean(pts, axis=0) for pts in groups.values()])


def stitch(
    frames: Sequence[np.ndarray],
    angles_deg: Sequence[float],
    voxel_size: float | None = None,
) -> np.ndarray:
    """将各雷达帧绕 Z 轴旋转其对应角度后拼接为完整点云。

    Args:
        frames: 每帧点云，形状 (n_i, 3)。
        angles_deg: 与 frames 一一对应的舵机角度（度）。
        voxel_size: 体素下采样边长（米），None 表示不下采样。

    Returns:
        拼接后的完整点云，形状 (N, 3)。
    """
    if len(frames) != len(angles_deg):
        raise ValueError(
            f"frames 数量 {len(frames)} 与 angles 数量 {len(angles_deg)} 不匹配"
        )

    transformed: list[np.ndarray] = []
    for frame, angle in zip(frames, angles_deg):
        if frame.shape[0] == 0:
            continue  # 空帧跳过
        rot = _rotation_z(float(angle))
        transformed.append(frame @ rot.T)  # 行向量右乘旋转矩阵

    if not transformed:
        return np.empty((0, 3))

    merged = np.concatenate(transformed, axis=0)
    if voxel_size is not None and voxel_size > 0.0:
        merged = _voxel_downsample(merged, voxel_size)
    return merged
