# 360° 两次扫描自标定修正雷达 Z 轴倾斜

## 背景

钻头杆扫描系统采用 LakiBeam 雷达横装：雷达系 **x 向下、y 向左、z 向前**。理想安装下，经 `InstallConfig.to_world` 映射后：

- 世界 x = 雷达 z（前，水平）
- 世界 y = 雷达 y（左，水平）
- 世界 z = -雷达 x（上，竖直）

实际机械安装存在微小倾斜，导致点云出现重影、分层。需要标定出雷达坐标系相对世界坐标系的精确旋转偏差（roll、pitch、yaw），并在生成点云时补偿。

## 目标

在不改变横装安装方式（x 下 / y 左 / z 前）的前提下，实现 360° 两次扫描自标定：

1. 转盘旋转 360°，同一环境被连续扫描两次。
2. 提取前后两次扫描中的对应特征（墙面边缘、天花板角点、地面平面）。
3. 构建优化问题，最小化对应点/平面对齐误差。
4. 使用 Levenberg-Marquardt 求解最优 roll、pitch、yaw。
5. 将标定结果写入安装配置，后续扫描自动补偿。

## 倾斜模型

实际安装姿态 = 理想横装姿态 `M_world` 叠加微小旋转 `R_tilt`：

```
p_world = (p_lidar @ M_world.T) @ R_tilt.T
```

`R_tilt` 用欧拉角表示：

```
R_tilt = Rz(yaw) · Ry(pitch) · Rx(roll)
```

- **roll**：绕雷达 x 轴（向下）旋转，影响世界 y-z 平面。
- **pitch**：绕雷达 y 轴（向左）旋转，影响世界 x-z 平面。
- **yaw**：绕雷达 z 轴（向前）旋转，影响世界 x-y 平面。

对于“Z 轴倾斜”问题，roll 和 pitch 是主要自由度；yaw 反映雷达绕前向轴的扭转，通常由安装对齐保证，但标定中仍一并估计。

## 标定原理

### 数据分半

将 360° 扫描点云按转盘角度分为两半：

- 前半圈：`θ ∈ [0°, 180°]`
- 后半圈：`θ ∈ [180°, 360°]`

若 `R_tilt = I`，同一空间特征在前、后半圈经理想变换后应完全重合。存在倾斜时，两半圈点云错位。

### 代价函数

采用**点到平面 ICP**构建代价，避免点-点最近邻在稀疏/不均匀点云中错配：

```
E(roll, pitch, yaw) = Σ |n_i^T · (R_tilt · p_i - q_i)|²
```

- `p_i`：前半圈点云中的一点。
- `q_i`：后半圈点云中与 `p_i` 对应的近点。
- `n_i`：`q_i` 所在局部平面的法向量。

对 `p_i` 在后半圈点云中搜索 k 近邻，拟合局部平面得到 `n_i` 和平面中心 `q_i`。

### 优化求解

使用 `scipy.optimize.least_squares(..., method='lm')` 求解，初始值 `(0, 0, 0)`。

## 实现计划

### 新增文件

| 文件 | 职责 |
|------|------|
| `drill_rod_scanner/calibration/__init__.py` | 包初始化 |
| `drill_rod_scanner/calibration/tilt_calibration.py` | 核心标定算法：数据分半、局部平面拟合、LM 优化 |
| `scripts/calibrate_tilt.py` | 离线标定 CLI |
| `tests/test_tilt_calibration.py` | 合成数据单元测试 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `scripts/install_config.py` | 扩展 `InstallConfig`：新增 `tilt_roll_deg`、`tilt_pitch_deg`、`tilt_yaw_deg` 字段和 `tilt_matrix()` 方法 |
| `scripts/servo_sweep_scan.py` | 在 `to_world()` 之后叠加 `_INSTALL.tilt_matrix().T` |
| `configs/install_side_mount.yaml` | 添加默认 `tilt: {roll_deg: 0, pitch_deg: 0, yaw_deg: 0}` |

### 数据流

```
雷达极坐标 → scan_to_xy → 雷达系 xyz
    → mount_transform (棱镜相位)
    → to_world (理想横装)
    → tilt_matrix (标定得到的 roll/pitch/yaw)
    → 偏心校正 (offset-y/z)
    → 绕转盘轴旋转聚合
```

## 算法细节

### 1. 数据分半

输入为 `(N, 3)` 世界系点云及对应角度数组 `angles_deg`（长度 N，每点一个角度）。按角度区间将点云分成两个子集。

### 2. 体素降采样

为避免内存爆炸和加速近邻搜索，对两半圈点云分别做体素降采样（默认 0.02 m）。纯 numpy 实现，不依赖 open3d。

### 3. 局部平面拟合

对前半圈每个采样点 `p_i`，在后半圈降采样点云中搜索 k 近邻（k=10）。对邻域做 SVD：

```
U, S, Vt = svd(neighbors - centroid)
normal = Vt[-1]
```

法向量方向一致性处理：若 `normal[2] < 0` 则取反，确保法向量大致朝上。

### 4. LM 优化

残差函数：

```python
def residuals(params):
    roll, pitch, yaw = params
    R = euler_to_matrix(roll, pitch, yaw)
    aligned = front_half @ R.T
    # 对每个 p_i 找后半圈最近点 q_i 及其法向量 n_i
    diffs = aligned - q_i
    return (diffs * n_i).sum(axis=1)
```

使用 `scipy.optimize.least_squares(residuals, x0=[0,0,0], method='lm')`。

### 5. 输出配置

标定结果写入新的 YAML 配置文件：

```yaml
name: side-mount-calibrated
description: 横装 + 360° 自标定 tilt
mount:
  axis: z
  angle_deg: 90.0
to_world:
  x: z
  y: y
  z: -x
turntable_axis: z
tilt:
  roll_deg: 0.023
  pitch_deg: -0.315
  yaw_deg: 0.008
```

## 测试计划

### 合成数据测试

1. 生成一个理想圆柱/房间点云。
2. 应用已知 `R_tilt(roll=1°, pitch=-2°, yaw=0.5°)` 得到“倾斜扫描”点云。
3. 按 0°–180° / 180°–360° 分半，调用标定函数。
4. 验证估计的 roll/pitch/yaw 与真值误差 `< 0.05°`。

### 理想数据测试

无倾斜时，标定结果应接近 `(0, 0, 0)`，误差 `< 0.01°`。

### 配置回写测试

`InstallConfig` 保存/加载 tilt 参数后，`tilt_matrix()` 一致。

## 风险与边界

- **环境退化**：空旷或特征单一的环境会导致平面拟合退化。建议在房间、走廊等有明显几何结构的环境中标定。
- **角度不均匀**：舵机转速不均匀会引入角度误差。标定时建议使用 `--continuous` 模式并匀速转动。
- **yaw 不可观**：若场景绕前向轴对称，yaw 可能难以准确估计。可支持 `--fix-yaw` 选项固定 yaw=0。
- **计算量**：完整点云（百万级）需先降采样，否则 LM 迭代过慢。

## 后续扩展

- 在线标定：在 `servo_sweep_scan.py` 中增加 `--calibrate-tilt` 模式，扫描后直接写回配置。
- 增量标定：保存多次标定结果，取平均提高稳定性。
- 可视化：标定前后对比显示两半圈点云重合度。
