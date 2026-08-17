# 360° 两次扫描自标定修正雷达 Z 轴倾斜

## 背景

钻头杆扫描系统采用 LakiBeam 雷达横装：雷达系 **x 向下、y 向左、z 向前**。理想安装下，经 `InstallConfig.to_world` 映射后：

- 世界 x = 雷达 z（前，水平）
- 世界 y = 雷达 y（左，水平）
- 世界 z = -雷达 x（上，竖直）

实际机械安装存在微小倾斜，导致点云出现重影、分层。需要标定出转盘轴相对世界坐标系竖直轴的旋转偏差（roll、pitch），并在生成点云时补偿。

## 目标

在不改变横装安装方式（x 下 / y 左 / z 前）的前提下，实现 360° 自标定：

1. 转盘旋转 360° 采集完整点云。
2. 利用 360° 扫描点云关于实际转盘轴的 180° 旋转对称性。
3. 构建优化问题，寻找实际转盘轴方向。
4. 使用 Levenberg-Marquardt 求解，并折算为 roll、pitch。
5. 将标定结果以可读角度写入原安装配置，后续扫描自动补偿。

## 倾斜模型

转盘轴不完全竖直时，等价于在理想转盘旋转之后叠加一个微小旋转 `R_tilt`：

```
p_world = (p_lidar @ M_mount.T @ M_world.T @ R_turntable(θ).T) @ R_tilt.T
```

`R_tilt` 用欧拉角表示，把实际转盘轴对齐到世界 z 轴：

```
R_tilt = Rz(yaw) · Ry(pitch) · Rx(roll)
```

- **roll**：绕世界 x 轴旋转，影响前后方向倾斜。
- **pitch**：绕世界 y 轴旋转，影响左右方向倾斜。
- **yaw**：通常由安装对齐保证，标定中固定为 0。

对于“Z 轴倾斜 / 转盘轴不竖直”问题，roll 和 pitch 是主要自由度。

## 标定原理

360° 扫描的完整点云关于实际转盘轴具有 180° 旋转对称性：把点云绕实际转盘轴旋转 180°，应与自身重合。若假设轴为世界 z 轴，而实际轴有倾斜，则旋转 180° 后点云不重合。

### 对称轴优化

用两个角度参数化实际转盘轴方向 `a`（绕世界 x、y 轴的微小倾斜）。对候选轴 `a`，构造 180° 旋转矩阵 `R_a(180°)`，计算旋转后点云与原点点云的最近邻距离平方和：

```
E(a) = Σ min_q || R_a(180°) · p - q ||²
```

使用 Levenberg-Marquardt 最小化 `E(a)`，得到实际转盘轴 `a`，再折算为 roll/pitch。

### 优化求解

使用 `scipy.optimize.least_squares(..., method='lm')` 求解，初始轴为世界 z 轴。近邻搜索使用 `scipy.spatial.cKDTree` 加速。

## 实现计划

### 新增文件

| 文件 | 职责 |
|------|------|
| `drill_rod_scanner/calibration/__init__.py` | 包初始化 |
| `drill_rod_scanner/calibration/tilt_calibration.py` | 核心标定算法：180° 旋转对称轴估计、LM 优化 |
| `scripts/calibrate_tilt.py` | 离线标定 CLI，结果写回原 YAML |
| `tests/test_tilt_calibration.py` | 合成数据单元测试 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `scripts/install_config.py` | 扩展 `InstallConfig`：新增 `tilt_roll_deg`、`tilt_pitch_deg`、`tilt_yaw_deg` 字段和 `tilt_matrix()` 方法 |
| `scripts/servo_sweep_scan.py` | 在转盘旋转聚合之后应用 `_INSTALL.tilt_matrix().T` |
| `configs/install_side_mount.yaml` | 添加默认 `tilt: {roll_deg: 0, pitch_deg: 0, yaw_deg: 0}` |
| `pyproject.toml` | 新增 `scipy` 依赖 |
| `README.md` | 增加 360° 自标定使用说明 |

### 数据流

```
雷达极坐标 → scan_to_xy → 雷达系 xyz
    → mount_transform (棱镜相位)
    → to_world (理想横装)
    → 偏心校正 (offset-y/z)
    → 绕转盘轴旋转聚合
    → tilt_matrix.T (标定得到的 roll/pitch/yaw 修正)
```

## 算法细节

### 1. 体素降采样

为避免内存爆炸和加速近邻搜索，对输入点云做体素降采样（默认 0.02 m）。纯 numpy 实现，不依赖 open3d。

### 2. 对称轴参数化

实际转盘轴 `a` 用两个微小倾斜角表示：

```
a = normalize([sin(tilt_y), -sin(tilt_x), cos(tilt_x) * cos(tilt_y)])
```

其中 `tilt_x`、`tilt_y` 对应 roll、pitch。

### 3. 180° 旋转与近邻搜索

对候选轴 `a`，用罗德里格斯公式构造 `R_a(180°)`，将点云旋转后通过 `cKDTree` 搜索最近邻。残差为最近邻距离。

### 4. LM 优化

使用 `scipy.optimize.least_squares(..., method='lm')` 最小化残差，得到 `tilt_x`、`tilt_y`，再折算为 roll、pitch。

### 5. 输出配置

标定结果直接写回 `--install-config` 指定的原 YAML 配置文件，新增可读角度字段：

```yaml
tilt:
  roll_deg: 0.023
  pitch_deg: -0.315
  yaw_deg: 0.0
```

## 测试计划

### 合成数据测试

1. 在 LiDAR 扫描平面内生成一组物理点。
2. 对每个点，沿 0°–360° 均匀采样多个转盘角度，计算带 tilt 的世界坐标。
3. 调用标定函数，验证估计 roll/pitch 与真值误差 `< 0.05°`。

### 理想数据测试

无倾斜时，标定结果应接近 `(0, 0, 0)`，误差 `< 0.02°`。

### 配置回写测试

`InstallConfig` 保存/加载 tilt 参数后，`tilt_matrix()` 一致。

## 风险与边界

- **环境退化**：空旷或特征单一的场景会导致对称轴估计退化。建议在房间、走廊等有明显几何结构的环境中标定。
- **角度不均匀**：舵机转速不均匀会引入角度误差。标定时建议使用 `--continuous` 模式并匀速转动。
- **yaw 不可观**：标定固定 yaw=0，假设雷达绕前向轴的安装对齐良好。
- **计算量**：完整点云（百万级）需先降采样，否则近邻搜索过慢。

## 后续扩展

- 在线标定：在 `servo_sweep_scan.py` 中增加 `--calibrate-tilt` 模式，扫描后直接写回配置。
- 增量标定：保存多次标定结果，取平均提高稳定性。
- 可视化：标定前后对比显示点云对称性。
