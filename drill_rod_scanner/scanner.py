"""扫描编排器：控制舵机从 A 到 B 步进，逐角度采集雷达帧，拼接并导出。"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from drill_rod_scanner.stitching.stitcher import stitch


@dataclass
class ScanResult:
    """一次扫描的结果。"""

    angles_deg: list[float]
    frames: list[np.ndarray] = field(default_factory=list)
    empty_frames: int = 0
    cloud: np.ndarray | None = None


class Scanner:
    """编排舵机与雷达完成一次旋转扫描。"""

    def __init__(
        self,
        servo,
        lidar,
        from_deg: float,
        to_deg: float,
        step_deg: float,
        settle_time_s: float = 0.5,
        voxel_size: float | None = None,
    ) -> None:
        self.servo = servo
        self.lidar = lidar
        self.from_deg = float(from_deg)
        self.to_deg = float(to_deg)
        self.step_deg = float(step_deg)
        self.settle_time_s = float(settle_time_s)
        self.voxel_size = voxel_size

    def _angle_sequence(self) -> list[float]:
        """生成 [from_deg, to_deg] 的步进角度序列（含端点）。"""
        if self.step_deg <= 0.0:
            raise ValueError("step_deg 必须大于 0")
        n = int(round((self.to_deg - self.from_deg) / self.step_deg))
        if n <= 0:
            return [self.from_deg, self.to_deg]
        return [self.from_deg + i * self.step_deg for i in range(n + 1)]

    def scan(self) -> ScanResult:
        """执行完整扫描：A→B 步进采帧 → 拼接 → 返回结果。"""
        self.servo.connect()
        self.lidar.connect()
        try:
            result = ScanResult(angles_deg=self._angle_sequence())
            collected_angles: list[float] = []
            for angle in result.angles_deg:
                self.servo.set_angle(angle)
                if self.settle_time_s > 0.0:
                    time.sleep(self.settle_time_s)
                frame = self.lidar.get_frame()
                if frame.shape[0] == 0:
                    result.empty_frames += 1
                    continue
                result.frames.append(frame)
                collected_angles.append(angle)

            if result.frames:
                # 注意：仅用实际采到帧对应的角度，避免空帧跳过后角度错位
                result.cloud = stitch(
                    result.frames, collected_angles,
                    voxel_size=self.voxel_size,
                )
            else:
                result.cloud = np.empty((0, 3))
            return result
        finally:
            self.servo.close()
            self.lidar.close()
