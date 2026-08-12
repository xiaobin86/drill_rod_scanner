# drill_rod_scanner 骨架实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 搭建 drill_rod_scanner 项目骨架：分层模块化 Python 包，舵机/雷达串口驱动留协议接口（命令待用户后续补充），点云拼接算法完整实现并可单测，mock 串口跑通完整扫描流程。

**Architecture:** 驱动层（SerialServo/SerialLidar，接口签名定死、协议命令留 TODO）→ 编排层（Scanner 按角度步进采帧）→ 拼接层（Stitcher 绕 Z 轴旋转变换合并点云）。核心拼接逻辑零硬件依赖，测试用 mock 串口。

**Tech Stack:** Python 3.10+、pyserial、numpy、open3d、pyyaml、pytest。conda 环境 `drill_rod_scanner`。

---

### Task 1: 项目脚手架（pyproject / 包结构 / 配置 / README / gitignore）

**Files:**
- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `README.md`
- Create: `config/scanner.yaml`
- Create: `drill_rod_scanner/__init__.py`
- Create: `drill_rod_scanner/servo/__init__.py`
- Create: `drill_rod_scanner/lidar/__init__.py`
- Create: `drill_rod_scanner/stitching/__init__.py`
- Create: `tests/__init__.py`
- Create: `scripts/run_scan.py`（占位，Task 7 填充）

- [ ] **Step 1: 创建 conda 环境**

```bash
conda create -n drill_rod_scanner python=3.10 -y
conda activate drill_rod_scanner
pip install pyserial numpy open3d pyyaml pytest
```

预期：环境创建成功，五个包可导入。

- [ ] **Step 2: 创建 pyproject.toml**

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "drill-rod-scanner"
version = "0.1.0"
description = "Rotary LiDAR scanning: servo rotates from angle A to B, LiDAR captures point cloud frames, stitched into full cloud for drill rod localization"
requires-python = ">=3.10"
dependencies = [
    "pyserial>=3.5",
    "numpy>=1.24",
    "open3d>=0.17",
    "pyyaml>=6.0",
]

[project.optional-dependencies]
dev = ["pytest>=7.0"]

[tool.setuptools.packages.find]
include = ["drill_rod_scanner*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 3: 创建 .gitignore**

```gitignore
__pycache__/
*.py[cod]
*.egg-info/
dist/
build/
.venv/
output/
.pytest_cache/
```

- [ ] **Step 4: 创建 config/scanner.yaml**

```yaml
servo:
  port: /dev/ttyUSB0
  baudrate: 115200
  # 协议命令、角度换算待用户后续补充

lidar:
  port: /dev/ttyUSB1
  baudrate: 115200
  # 协议命令、点云帧解析待用户后续补充

scan:
  from_deg: 0.0        # 起始角度 A
  to_deg: 180.0        # 结束角度 B
  step_deg: 10.0       # 角度步进
  settle_time_s: 0.5   # 舵机到位后等待稳定的时间

stitch:
  voxel_size: 0.005    # 体素下采样边长（米），null 表示不下采样

output:
  dir: output/
  cloud_format: ply    # ply | pcd
```

- [ ] **Step 5: 创建包 __init__ 文件与目录**

```bash
mkdir -p drill_rod_scanner/servo drill_rod_scanner/lidar drill_rod_scanner/stitching tests scripts
touch drill_rod_scanner/__init__.py drill_rod_scanner/servo/__init__.py drill_rod_scanner/lidar/__init__.py drill_rod_scanner/stitching/__init__.py tests/__init__.py
```

`drill_rod_scanner/__init__.py` 内容：

```python
"""drill_rod_scanner: rotary LiDAR scan + point cloud stitching."""

__version__ = "0.1.0"
```

- [ ] **Step 6: 创建 README.md**

```markdown
# drill_rod_scanner

钻头杆点云扫描定位程序。舵机带动激光雷达绕竖直轴旋转，从角度 A 扫到角度 B，
按步进角度逐帧采集点云，按舵机角度旋转变换后拼接为完整点云。

## 快速开始

```bash
conda activate drill_rod_scanner
python scripts/run_scan.py --config config/scanner.yaml
```

## 目录结构

- `drill_rod_scanner/servo/` 舵机串口驱动（协议命令待补充）
- `drill_rod_scanner/lidar/` 雷达串口驱动（协议命令待补充）
- `drill_rod_scanner/stitching/` 点云拼接算法
- `drill_rod_scanner/scanner.py` 扫描编排器
- `config/scanner.yaml` 运行配置
```

- [ ] **Step 7: 验证结构**

```bash
python -c "import drill_rod_scanner; print(drill_rod_scanner.__version__)"
```

预期：输出 `0.1.0`。

- [ ] **Step 8: Commit**

```bash
git add pyproject.toml .gitignore README.md config/ drill_rod_scanner/ tests/
git commit -m "chore: 项目脚手架（包结构/配置/README）"
```

---

### Task 2: Stitcher 拼接算法（TDD，零硬件依赖）

**Files:**
- Create: `drill_rod_scanner/stitching/stitcher.py`
- Test: `tests/test_stitcher.py`

- [ ] **Step 1: 写失败测试**

`tests/test_stitcher.py`：

```python
import numpy as np
import pytest

from drill_rod_scanner.stitching.stitcher import stitch


def test_single_frame_angle_zero_unchanged():
    frame = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.5]])
    out = stitch([frame], [0.0])
    np.testing.assert_allclose(out, frame, atol=1e-9)


def test_single_frame_rotated_90_deg_around_z():
    # 绕 Z 轴旋转 90°：x->y, y->-x
    frame = np.array([[1.0, 0.0, 0.0]])
    out = stitch([frame], [90.0])
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_two_frames_merged():
    f0 = np.array([[1.0, 0.0, 0.0]])
    f90 = np.array([[0.0, 1.0, 0.0]])
    out = stitch([f0, f90], [0.0, 90.0])
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out[0], [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out[1], [-1.0, 0.0, 0.0], atol=1e-9)


def test_empty_frame_skipped():
    frame = np.empty((0, 3))
    out = stitch([frame], [0.0])
    assert out.shape == (0, 3)


def test_voxel_downsample():
    # 4 个点挤在一个 1cm 体素内，voxel_size=0.02 合并为 1 个点
    frame = np.array([
        [0.0, 0.0, 0.0],
        [0.001, 0.0, 0.0],
        [0.0, 0.001, 0.0],
        [0.0, 0.0, 0.001],
    ])
    out = stitch([frame], [0.0], voxel_size=0.02)
    assert out.shape[0] == 1


def test_angle_list_length_mismatch_raises():
    with pytest.raises(ValueError):
        stitch([np.zeros((1, 3))], [0.0, 10.0])
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_stitcher.py -v`
预期：FAIL，`ImportError: cannot import name 'stitch'`。

- [ ] **Step 3: 实现 stitcher.py**

`drill_rod_scanner/stitching/stitcher.py`：

```python
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
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_stitcher.py -v`
预期：PASS，6 个测试全部通过。

- [ ] **Step 5: Commit**

```bash
git add drill_rod_scanner/stitching/stitcher.py tests/test_stitcher.py
git commit -m "feat(stitching): 实现按角度旋转变换拼接点云，支持体素下采样"
```

---

### Task 3: 舵机串口驱动接口（协议命令留 TODO）

**Files:**
- Create: `drill_rod_scanner/servo/serial_servo.py`
- Test: `tests/test_serial_servo.py`

- [ ] **Step 1: 写失败测试**

`tests/test_serial_servo.py`：

```python
import pytest

from drill_rod_scanner.servo.serial_servo import SerialServo, ServoConnectionError


def test_connect_missing_port_raises():
    servo = SerialServo(port="/dev/does-not-exist", baudrate=115200)
    with pytest.raises(ServoConnectionError):
        servo.connect()


def test_close_without_connect_is_safe():
    servo = SerialServo(port="/dev/ttyUSB0", baudrate=115200)
    servo.close()  # 不应抛异常
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_serial_servo.py -v`
预期：FAIL，`ImportError`。

- [ ] **Step 3: 实现 serial_servo.py**

`drill_rod_scanner/servo/serial_servo.py`：

```python
"""舵机串口驱动。

接口签名已定死（connect/set_angle/read_angle/close），
具体的串口协议命令由用户后续补充（见下方 TODO 标记）。
"""

from __future__ import annotations

import serial


class ServoConnectionError(Exception):
    """舵机串口连接失败。"""


class ServoTimeoutError(Exception):
    """舵机响应超时。"""


class SerialServo:
    """通过串口控制舵机角度。"""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        """打开串口。"""
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            raise ServoConnectionError(f"无法连接舵机串口 {self.port}: {e}") from e

        # TODO(用户补充): 舵机串口协议命令 ——
        #  例：发送查询/握手指令确认舵机在线，如 b"\xAA\x55..."。

    def set_angle(self, angle_deg: float) -> None:
        """设置舵机目标角度（度）。"""
        self._require_connected()
        # TODO(用户补充): 协议命令 ——
        #  将 angle_deg 按舵机量程换算为串口指令字节并发送。
        raise NotImplementedError(
            "舵机 set_angle 串口协议命令待用户补充"
        )

    def read_angle(self) -> float:
        """读取舵机当前角度（度）。"""
        self._require_connected()
        # TODO(用户补充): 协议命令 ——
        #  发送查询指令，解析返回的角度字节。
        raise NotImplementedError(
            "舵机 read_angle 串口协议命令待用户补充"
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _require_connected(self) -> None:
        if self._ser is None:
            raise ServoConnectionError("舵机串口未连接，请先调用 connect()")
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_serial_servo.py -v`
预期：PASS，2 个测试通过。

- [ ] **Step 5: Commit**

```bash
git add drill_rod_scanner/servo/serial_servo.py tests/test_serial_servo.py
git commit -m "feat(servo): 舵机串口驱动接口骨架，协议命令留 TODO"
```

---

### Task 4: 雷达串口驱动接口（协议命令留 TODO）

**Files:**
- Create: `drill_rod_scanner/lidar/serial_lidar.py`
- Test: `tests/test_serial_lidar.py`

- [ ] **Step 1: 写失败测试**

`tests/test_serial_lidar.py`：

```python
import pytest

from drill_rod_scanner.lidar.serial_lidar import LidarConnectionError, SerialLidar


def test_connect_missing_port_raises():
    lidar = SerialLidar(port="/dev/does-not-exist", baudrate=115200)
    with pytest.raises(LidarConnectionError):
        lidar.connect()


def test_close_without_connect_is_safe():
    lidar = SerialLidar(port="/dev/ttyUSB1", baudrate=115200)
    lidar.close()  # 不应抛异常
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_serial_lidar.py -v`
预期：FAIL，`ImportError`。

- [ ] **Step 3: 实现 serial_lidar.py**

`drill_rod_scanner/lidar/serial_lidar.py`：

```python
"""雷达串口驱动。

接口签名已定死（connect/get_frame/close），
具体的串口协议命令与点云帧解析由用户后续补充（见下方 TODO 标记）。
"""

from __future__ import annotations

import serial
import numpy as np


class LidarConnectionError(Exception):
    """雷达串口连接失败。"""


class LidarTimeoutError(Exception):
    """雷达响应超时。"""


class SerialLidar:
    """通过串口采集雷达点云帧。"""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        """打开串口。"""
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            raise LidarConnectionError(f"无法连接雷达串口 {self.port}: {e}") from e

        # TODO(用户补充): 雷达串口协议命令 ——
        #  例：发送启动扫描指令，如 b"\xA5\x20..."。

    def get_frame(self) -> np.ndarray:
        """采集一帧点云，返回形状 (n, 3) 的 numpy 数组（单位：米）。

        返回的坐标系为雷达自身坐标系（扫描平面内 x-y 为水平面，z 为高度）。
        """
        self._require_connected()
        # TODO(用户补充): 协议命令与帧解析 ——
        #  发送单帧采集指令，读取并解析距离/角度数据，
        #  转换为 (n, 3) 直角坐标点云。
        raise NotImplementedError(
            "雷达 get_frame 串口协议命令待用户补充"
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _require_connected(self) -> None:
        if self._ser is None:
            raise LidarConnectionError("雷达串口未连接，请先调用 connect()")
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_serial_lidar.py -v`
预期：PASS，2 个测试通过。

- [ ] **Step 5: Commit**

```bash
git add drill_rod_scanner/lidar/serial_lidar.py tests/test_serial_lidar.py
git commit -m "feat(lidar): 雷达串口驱动接口骨架，协议命令留 TODO"
```

---

### Task 5: 扫描编排器（TDD，mock 串口跑通全流程）

**Files:**
- Create: `drill_rod_scanner/scanner.py`
- Create: `drill_rod_scanner/io.py`
- Test: `tests/test_scanner.py`

- [ ] **Step 1: 写失败测试**

`tests/test_scanner.py`：

```python
import time

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
    assert len(result.frames) == 3  # 空帧被跳过
    assert result.cloud.shape == (0, 3)
    assert result.empty_frames == 3
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_scanner.py -v`
预期：FAIL，`ImportError`。

- [ ] **Step 3: 实现 scanner.py**

`drill_rod_scanner/scanner.py`：

```python
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
```

- [ ] **Step 4: 实现 io.py（点云导出）**

`drill_rod_scanner/io.py`：

```python
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
```

- [ ] **Step 5: 运行测试确认通过**

Run: `pytest tests/test_scanner.py -v`
预期：PASS，5 个测试通过。

- [ ] **Step 6: Commit**

```bash
git add drill_rod_scanner/scanner.py drill_rod_scanner/io.py tests/test_scanner.py
git commit -m "feat(scanner): 扫描编排器 + 点云导出，mock 串口测试通过"
```

---

### Task 6: CLI 入口 run_scan.py

**Files:**
- Modify: `scripts/run_scan.py`
- Create: `tests/test_cli.py`

- [ ] **Step 1: 写失败测试**

`tests/test_cli.py`：

```python
import sys
from pathlib import Path

from drill_rod_scanner.cli import load_config, run_from_config


def test_load_config(tmp_path):
    cfg = tmp_path / "scanner.yaml"
    cfg.write_text(
        "servo:\n"
        "  port: /dev/ttyUSB0\n"
        "  baudrate: 115200\n"
        "lidar:\n"
        "  port: /dev/ttyUSB1\n"
        "  baudrate: 115200\n"
        "scan:\n"
        "  from_deg: 0.0\n"
        "  to_deg: 20.0\n"
        "  step_deg: 10.0\n"
        "  settle_time_s: 0.0\n"
        "stitch:\n"
        "  voxel_size: null\n"
        "output:\n"
        "  dir: out\n"
        "  cloud_format: ply\n"
    )
    cfg_obj = load_config(cfg)
    assert cfg_obj["servo"]["port"] == "/dev/ttyUSB0"
    assert cfg_obj["scan"]["from_deg"] == 0.0
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_cli.py -v`
预期：FAIL，`ImportError`。

- [ ] **Step 3: 实现 cli.py**

`drill_rod_scanner/cli.py`：

```python
"""CLI 辅助：配置加载与扫描执行。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from drill_rod_scanner.io import save_result
from drill_rod_scanner.lidar.serial_lidar import SerialLidar
from drill_rod_scanner.scanner import Scanner
from drill_rod_scanner.servo.serial_servo import SerialServo


def load_config(path: str | Path) -> dict[str, Any]:
    """从 YAML 文件加载配置。"""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_from_config(cfg: dict[str, Any]) -> None:
    """按配置执行扫描并导出结果。"""
    servo = SerialServo(
        port=cfg["servo"]["port"],
        baudrate=cfg["servo"]["baudrate"],
    )
    lidar = SerialLidar(
        port=cfg["lidar"]["port"],
        baudrate=cfg["lidar"]["baudrate"],
    )
    scan_cfg = cfg["scan"]
    stitch_cfg = cfg.get("stitch", {})
    scanner = Scanner(
        servo=servo,
        lidar=lidar,
        from_deg=scan_cfg["from_deg"],
        to_deg=scan_cfg["to_deg"],
        step_deg=scan_cfg["step_deg"],
        settle_time_s=scan_cfg.get("settle_time_s", 0.5),
        voxel_size=stitch_cfg.get("voxel_size"),
    )
    result = scanner.scan()
    out_cfg = cfg.get("output", {})
    files = save_result(
        result,
        output_dir=out_cfg.get("dir", "output"),
        cloud_format=out_cfg.get("cloud_format", "ply"),
    )
    print(f"扫描完成: {len(result.frames)} 帧, {result.empty_frames} 空帧跳过")
    for kind, path in files.items():
        print(f"  {kind}: {path}")
```

- [ ] **Step 4: 填充 scripts/run_scan.py**

`scripts/run_scan.py`：

```python
#!/usr/bin/env python3
"""drill_rod_scanner CLI 入口。

用法:
    python scripts/run_scan.py [--config config/scanner.yaml]
"""

from __future__ import annotations

import argparse

from drill_rod_scanner.cli import load_config, run_from_config


def main() -> None:
    parser = argparse.ArgumentParser(description="钻头杆旋转扫描点云采集")
    parser.add_argument(
        "--config", default="config/scanner.yaml",
        help="配置文件路径（默认 config/scanner.yaml）",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    run_from_config(cfg)


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 运行测试确认通过**

Run: `pytest tests/test_cli.py -v`
预期：PASS，1 个测试通过。

- [ ] **Step 6: 全量测试 + 语法检查**

Run: `pytest tests/ -v`
预期：全部 PASS（stitcher 6 + servo 2 + lidar 2 + scanner 5 + cli 1 = 16 个）。

Run: `python scripts/run_scan.py --help`
预期：打印用法帮助（不连接硬件）。

- [ ] **Step 7: Commit**

```bash
git add drill_rod_scanner/cli.py scripts/run_scan.py tests/test_cli.py
git commit -m "feat(cli): 命令行入口与配置加载"
```

---

### Task 7: 最终验证与收尾

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 更新 README 补充测试说明**

在 README 快速开始节后追加：

```markdown
## 测试

```bash
conda activate drill_rod_scanner
pytest tests/ -v
```

## 当前状态

- 点云拼接算法已完整实现并通过单测（不依赖硬件）。
- 舵机/雷达串口协议命令待补充：`drill_rod_scanner/servo/serial_servo.py`
  与 `drill_rod_scanner/lidar/serial_lidar.py` 中的 `TODO(用户补充)` 标记处。
```

- [ ] **Step 2: 全量测试确认**

Run: `pytest tests/ -v`
预期：16 个测试全部 PASS。

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: 补充测试说明与当前状态"
```

- [ ] **Step 4: 检查 git 状态干净**

```bash
git status --short && git log --oneline
```

预期：无未提交改动，提交记录完整（约 7 个 commit）。
