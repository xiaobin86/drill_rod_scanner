# 360° 联合自标定：转盘轴倾斜 + LiDAR 安装偏航

## 背景

钻头杆扫描系统采用 LakiBeam 雷达横装：雷达系 **x 向下、y 向左、z 向前**。理想安装下，经 `InstallConfig.to_world` 映射后：

- 世界 x = 雷达 z（前，水平）
- 世界 y = 雷达 y（左，水平）
- 世界 z = -雷达 x（上，竖直）

实际机械安装存在两类误差，导致点云出现重影、分层或地面倾斜：

1. **转盘轴不竖直**：转盘轴相对世界 z 轴有微小 roll/pitch 倾斜。
2. **LiDAR 绕自身 z 轴扭转**：扫描平面偏离竖直，地面点高度散开。

需要同时标定这两类参数，并在生成点云时补偿。

## 目标

在不改变横装安装方式（x 下 / y 左 / z 前）的前提下，实现单次 360° 联合自标定：

1. 转盘旋转 360° 采集完整点云。
2. 利用 360° 扫描点云关于实际转盘轴的 180° 旋转对称性。
3. 同时估计转盘轴倾斜（roll/pitch）和 LiDAR 安装偏航（yaw）。
4. 将标定结果以可读角度写入原安装配置，后续扫描自动补偿。

## 误差模型

### 转盘轴倾斜

转盘轴不完全竖直时，真实世界坐标点应先绕实际转盘轴旋转、再投影回世界系。
`R_tilt` 把实际转盘轴对齐到世界 z 轴：

```
R_tilt = Rz(yaw) · Ry(pitch) · Rx(roll)
```

- **roll**：绕世界 x 轴旋转，影响前后方向倾斜。
- **pitch**：绕世界 y 轴旋转，影响左右方向倾斜。
- **yaw**：通常由安装对齐保证，标定中固定为 0。

### LiDAR 安装偏差

LiDAR 安装偏差 `R_lidar` 描述理想雷达坐标系到实际雷达坐标系的旋转：

```
R_lidar = Rz(yaw) · Ry(pitch) · Rx(roll)
```

在横装约定下，雷达 x 轴指向世界 z 轴，因此 `R_lidar.roll` 等价于让整团点云绕竖直轴旋转一个全局偏航，单次 360° 扫描不可观；`R_lidar.pitch` 与 `tilt.pitch` 高度耦合，程序固定为 0。只有 `R_lidar.yaw` 可标定。

### 完整校正公式

对每个在转盘角度 θ 下测得的点 `p`：

```
p_corrected = p
    @ R_z(θ)                       # 撤销理想转盘旋转
    @ M.T @ R_lidar @ M            # 撤销 LiDAR 安装偏差（转到世界系）
    @ R_tilt @ R_z(θ).T @ R_tilt.T # Redo 绕真实转盘轴旋转
```

其中 `M = M_mount.T @ M_world` 是雷达系到世界系的复合旋转。

## 标定原理

360° 扫描的完整点云关于实际转盘轴具有 180° 旋转对称性：同一物理点会在角度 θ 和 θ+180° 被扫描到两次，校正后应重合。

### 1. 对侧点重合约束

对每个在角度 θ 测得的点，在 θ+180° 的对侧扫描中寻找最近邻点，计算两者校正后的距离：

```
E_sym = Σ || p_corrected(θ) - p_corrected(θ+180°) ||²
```

该约束可估计转盘轴倾斜。

### 2. 地面平整约束

若 LiDAR 绕自身 z 轴扭转，扫描面不再竖直，地面点会上下散开。取点云中 z 值最低的 30% 作为地面点，最小化其 z 值标准差：

```
E_flat = std(ground_z)
```

该约束可估计 `lidar_tilt.yaw`。

### 3. 交替优化

由于两类参数存在耦合，采用交替策略：

1. 固定 `tilt = I`，用地面平整约束估计 `lidar_tilt.yaw`。
2. 固定 `lidar_tilt.yaw`，用对侧点重合约束 + 地面平整约束估计 `tilt.roll/pitch`。
3. 重复 2–3 轮，直到收敛。

每轮内部使用粗网格 + L-BFGS-B 精化。

## 实现计划

### 新增/核心文件

| 文件 | 职责 |
|------|------|
| `drill_rod_scanner/calibration/joint_tilt_calibration.py` | 联合标定算法：对侧点重合 + 地面平整 + 交替优化 |
| `scripts/verify_tilt_calibration.py` | 真值仿真验证脚本：用配置中的 tilt/lidar_tilt 作为真值，仿真 360° 扫描后运行标定，输出误差 |
| `tests/test_joint_tilt_calibration.py` | 合成数据回归测试：yaw-only、tilt-only、target case |
| `drill_rod_scanner/calibration/tilt_calibration.py` | 旧版转盘轴倾斜标定（保留兼容） |
| `scripts/calibrate_tilt.py` | 旧版离线标定 CLI（保留兼容） |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `scripts/install_config.py` | 扩展 `InstallConfig`：新增 `lidar_tilt_*_deg`、`offset_*_m` 字段和 `lidar_tilt_matrix()` 方法 |
| `scripts/servo_sweep_scan.py` | 默认加载 `configs/install_side_mount.yaml`；应用 `lidar_tilt_matrix().T` 和 `tilt_matrix().T` |
| `configs/install_side_mount.yaml` | 添加默认 `tilt`、`lidar_tilt`、`offset` 字段 |
| `README.md` | 增加联合标定与真值验证使用说明 |
| `docs/install_config.md` | 补充 tilt / lidar_tilt / offset 字段说明 |

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

为避免内存爆炸和加速近邻搜索，对输入点云做体素降采样（默认 0.05 m）。纯 numpy 实现，不依赖 open3d。

### 2. 对侧角度配对

不采用整团点云绕 z 轴旋转 180° 再找最近邻，而是按每个测量角度 θ 直接到 θ+180° 的对侧帧中找最近邻。这样更贴合扫描物理过程，避免不同角度点互相干扰。

### 3. 对称残差

对每个 θ，构造源点集 `S(θ)` 和目标点集 `S(θ+180°)`，在校正后计算最近邻距离：

```
residuals = cKDTree(S_corrected(θ+180°)).query(S_corrected(θ), k=1)[0]
```

### 4. 地面平整代价

取校正后点云中 z 值最低的 30% 点，计算标准差：

```
flatness = std(z_lowest_30_percent)
```

### 5. L-BFGS-B 优化

- `tilt.roll/pitch`：在 9×9 网格上粗搜，再用 L-BFGS-B 精化；代价为 `sum(residuals²) + 1e4 * flatness`。
- `lidar_tilt.yaw`：在 51 点网格上粗搜，再用 L-BFGS-B 精化；代价为 `flatness`。

### 6. 输出配置

标定结果直接写回 `--install-config` 指定的原 YAML 配置文件：

```yaml
tilt:
  roll_deg: 0.023
  pitch_deg: -0.315
  yaw_deg: 0.0
lidar_tilt:
  roll_deg: 0.0
  pitch_deg: 0.0
  yaw_deg: 1.512
offset:
  y_m: -0.055
  z_m: 0.025
```

## 测试计划

### 合成数据回归测试

在 `tests/test_joint_tilt_calibration.py` 中覆盖三类真值：

1. **lidar_tilt.yaw only**：验证 `lidar_tilt.yaw` 误差 `< 0.05°`。
2. **tilt.roll/pitch only**：验证 `tilt.roll/pitch` 误差分别 `< 0.3°` / `< 0.1°`。
3. **target case**：`tilt=0`、`lidar_tilt.roll=1.5°/yaw=1.5°`；验证 `tilt` 与 `lidar_tilt.yaw` 误差 `< 0.05°`，`lidar_tilt.roll` 固定为 0。

### 真值验证脚本

`scripts/verify_tilt_calibration.py` 以配置文件为真值，仿真 360° 扫描并运行标定，打印误差，用于快速评估标定可靠性。

### 配置回写测试

`InstallConfig` 保存/加载 `tilt` / `lidar_tilt` / `offset` 参数后，矩阵计算一致。

## 风险与边界

- **环境退化**：空旷或特征单一的场景会导致对称轴估计退化。建议在房间、走廊等有明显几何结构的环境中标定。
- **角度不均匀**：舵机转速不均匀会引入角度误差。标定时建议使用匀速转动。
- **lidar_tilt.roll 不可观**：在横装约定下等价于全局偏航，单次 360° 扫描无法估计，程序固定为 0。
- **lidar_tilt.pitch 固定为 0**：为避免与 `tilt.pitch` 耦合，当前实现不估计该参数。
- **计算量**：完整点云（百万级）需先降采样，否则近邻搜索过慢。
- **与 `servo_sweep_scan.py` 的 tilt 应用方式尚未对齐**：当前扫描脚本仍使用旧的 `rotated @ tilt_matrix().T` 后处理，与新模块的物理模型存在差异，后续需要统一。

## 后续扩展

- 在线标定：在 `servo_sweep_scan.py` 中增加 `--calibrate-tilt` 模式，扫描后直接写回配置。
- 增量标定：保存多次标定结果，取平均提高稳定性。
- 可视化：标定前后对比显示点云对称性。
- 外部参照：引入已知水平/竖直方向的标定板，使 `lidar_tilt.roll` 可观。
- 统一扫描流程中的 tilt 应用方式，与 `joint_tilt_calibration` 的物理模型一致。
