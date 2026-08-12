import numpy as np
import pytest

from drill_rod_scanner.scanner import ScanResult, Scanner


class MockServo:
    def __init__(self) -> None:
        self.current = 0.0
        self.angles: list[float] = []

    def connect(self) -> None:
        pass

    def set_angle(self, angle_deg: float) -> None:
        self.current = angle_deg
        self.angles.append(angle_deg)

    def read_angle(self) -> float:
        return self.current

    def close(self) -> None:
        pass


class MockLidar:
    def __init__(self) -> None:
        self.frames: list[np.ndarray] = []

    def connect(self) -> None:
        pass

    def get_frame(self) -> np.ndarray:
        self.frames.append(np.array([[0.5, 0.0, 0.2]]))
        return self.frames[-1]

    def close(self) -> None:
        pass


def make_scanner() -> Scanner:
    return Scanner(
        servo=MockServo(),
        lidar=MockLidar(),
        from_deg=0.0,
        to_deg=20.0,
        step_deg=10.0,
        settle_time_s=0.0,
        voxel_size=None,
    )


def test_scan_angle_sequence():
    scanner = make_scanner()
    result = scanner.scan()
    assert scanner.servo.angles == [0.0, 10.0, 20.0]
    assert result.angles_deg == [0.0, 10.0, 20.0]


def test_scan_collects_one_frame_per_angle():
    scanner = make_scanner()
    result = scanner.scan()
    assert len(result.frames) == 3
    assert result.frames[0].shape == (1, 3)


def test_scan_stitches_cloud():
    scanner = make_scanner()
    result = scanner.scan()
    assert result.cloud.shape == (3, 3)


def test_scan_step_larger_than_range_single_angle():
    scanner = Scanner(
        servo=MockServo(), lidar=MockLidar(),
        from_deg=0.0, to_deg=10.0, step_deg=30.0,
        settle_time_s=0.0, voxel_size=None,
    )
    result = scanner.scan()
    assert result.angles_deg == [0.0, 10.0]


def test_scan_empty_frame_skipped():
    class EmptyLidar(MockLidar):
        def get_frame(self) -> np.ndarray:
            self.frames.append(np.empty((0, 3)))
            return self.frames[-1]

    scanner = Scanner(
        servo=MockServo(), lidar=EmptyLidar(),
        from_deg=0.0, to_deg=20.0, step_deg=10.0,
        settle_time_s=0.0, voxel_size=None,
    )
    result = scanner.scan()
    assert len(scanner.lidar.frames) == 3  # 雷达被询问 3 次
    assert len(result.frames) == 0         # 空帧全部被跳过，未收集
    assert result.cloud.shape == (0, 3)
    assert result.empty_frames == 3
