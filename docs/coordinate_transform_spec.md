# 雷达安装坐标变换规范

> 本文件长期有效，定义 mount / to_world 配置的物理意义，以及整个系统的
> 坐标变换链。所有代码（扫描、标定、仿真、可视化）必须遵循本文档的约定。

## 1. 两个配置项的意义

雷达安装姿态由 **两个独立自由度** 描述，二者不能混淆：

| 配置项 | 物理意义 | 作用对象 | 影响 |
|--------|---------|---------|------|
| `mount` | 棱镜 0° 参考相位 | 扫描**数据点** | 绕雷达系某轴旋转点云，对齐出厂 0° 参考 |
| `to_world` | 安装姿态（雷达系→世界系轴映射） | 雷达**物理方向** | 定义雷达三轴在世界系中的指向 |

### 1.1 mount（棱镜 0° 参考相位）

LakiBeam 出厂时，扫描弧的 0° 参考方向与雷达 x 轴存在固定夹角（通常 90°）。
`mount` 在**雷达系内**绕指定轴旋转扫描数据，把这个 0° 参考对齐到约定方向。

```yaml
mount:
  axis: z        # 绕雷达系哪个轴旋转
  angle_deg: 90.0  # 旋转角度（度）
```

实现（`install_config.py`）：

```python
def mount_matrix(self) -> np.ndarray:
    return rotation_matrix(self.mount_axis, self.mount_angle_deg)

def mount_transform(self, points):
    return points @ self.mount_matrix().T   # 行向量，绕雷达z轴转-90°
```

**关键**：`mount` 旋转的是**扫描数据点**，不改变雷达物理安装方向。
雷达坐标系的物理指向完全由 `to_world` 决定。

### 1.2 to_world（安装姿态）

定义雷达系三轴在世界坐标系中的指向，用"世界 x/y/z 各取自雷达系哪个轴"描述：

```yaml
to_world:
  x: z    # 世界 x 轴 = 雷达 z 轴（前）
  y: y    # 世界 y 轴 = 雷达 y 轴（左）
  z: -x   # 世界 z 轴 = -雷达 x 轴（上），即雷达 x 轴朝下
```

实现（`install_config.py`）：

```python
def to_world_matrix(self) -> np.ndarray:
    return np.column_stack([
        _axis_vector(self.world_x),  # 世界 x 轴在雷达系的坐标
        _axis_vector(self.world_y),  # 世界 y 轴在雷达系的坐标
        _axis_vector(self.world_z),  # 世界 z 轴在雷达系的坐标
    ])
```

`to_world_matrix` 的三行恰好是雷达三轴在世界系的方向（未旋转时）：

```python
tw = cfg.to_world_matrix()
雷达x轴方向 = tw[0]   # 横装 = [0, 0, -1] 世界-z（朝下）
雷达y轴方向 = tw[1]   # 横装 = [0, 1, 0]  世界y（朝左）
雷达z轴方向 = tw[2]   # 横装 = [1, 0, 0]  世界x（朝前）
```

## 2. 完整坐标变换链（横装 side-mount 为例）

### 2.1 正向变换（扫描：雷达系 → 世界系）

`servo_sweep_scan.py::process_frame`：

```python
frame = frame @ _INSTALL.lidar_tilt_matrix().T   # ① 雷达安装偏差修正（雷达系）
frame = mount_transform(frame)                    # ② 棱镜相位（雷达系）
frame[:, 1] += args.offset_y                      # ③ 偏心（雷达系）
frame[:, 2] += args.offset_z
frame = to_world(frame)                           # ④ 安装姿态（雷达系→世界系）
rotated = rotate_points(frame, args.axis, angle)  # ⑤ 舵机旋转（世界系）
rotated = rotated @ _INSTALL.tilt_matrix().T      # ⑥ 转盘轴倾斜修正（世界系）
```

数学形式（行向量）：

```
p_world = p_radar @ lidar_tilt.T @ mount.T @ to_world @ Rz(θ).T @ tilt.T
```

其中 `m = mount.T @ to_world`（棱镜相位 + 安装姿态的复合），可简写为：

```
p_world = p_radar @ lidar_tilt.T @ m @ Rz(θ).T @ tilt.T
```

**通俗解读（从左往右读，先发生的变换在左边）**：

行向量约定 `p @ A @ B @ C` 表示先应用 A、再 B、再 C，即**从左往右读**。
以"雷达测到的点在传送带上被逐步搬运到世界坐标系"理解：

| 顺序 | 变换 | 通俗解释 | 坐标系 |
|------|------|---------|--------|
| 起点 | `p` | 雷达报出点的坐标 (x,y,z) | 雷达系 |
| 1️⃣ | `@ lidar_tilt.T` | 修雷达安装歪了的偏差（装正） | 雷达系 |
| 2️⃣ | `@ mount.T` | 对齐棱镜 0° 参考相位 | 雷达系 |
| 3️⃣ | `@ to_world` | 从"雷达视角"切换到"世界（房间）视角"（轴映射） | 雷达系→世界系 |
| 4️⃣ | `@ Rz(θ).T` | 转盘转到 θ°，点跟着绕竖直轴旋转 | 世界系 |
| 5️⃣ | `@ tilt.T` | 修正转盘轴本身的倾斜 | 世界系 |
| 终点 | `p_world` | 点在世界坐标系中的真实位置 | 世界系 |

要点：
- **前两步在雷达系内**：修雷达自身的毛病（装歪、相位偏），点还没离开雷达坐标系。
- **第三步是坐标系切换**：`to_world` 做轴映射（雷达 z→世界 x、雷达 y→世界 y、雷达 -x→世界 z），此后点进入世界系。
- **后两步在世界系内**：转盘旋转与轴倾斜都是世界坐标系下的运动。
- `.T`（转置）表示反向/撤销：标定时的 `m.T` 即世界→雷达（退回去），`Rz(θ).T` 即把已旋转的点转回去。

**一句话**：先修雷达自身偏差（lidar_tilt、mount）→ 换到世界坐标系（to_world）→ 再叠加转盘运动（旋转 + 倾斜修正）。

### 2.2 逆向校正（标定：世界系 → 校正后位置）

`joint_tilt_calibration.py::_corrected_cloud`：

```python
corrected[mask] = (
    points[mask] @ r_theta
    @ m.T @ r_lidar_tilt @ m
    @ r_tilt @ r_theta.T @ r_tilt.T
)
```

数学形式：

```
p_corrected = p @ Rz(θ) @ m.T @ R_lidar @ m @ R_tilt @ Rz(θ).T @ R_tilt.T
```

含义：
1. `p @ Rz(θ)`：撤销舵机旋转（世界系→雷达系方向）
2. `@ m.T`：撤销安装姿态（世界系→雷达系）
3. `@ R_lidar`：应用 lidar 安装偏差校正（**雷达系内**）
4. `@ m`：重新应用安装姿态（雷达系→世界系）
5. `@ R_tilt @ Rz(θ).T @ R_tilt.T`：绕实际转盘轴重新旋转（转盘轴倾斜校正）

其中 `m = mount.T @ to_world`，R_lidar 在雷达系内应用，R_tilt 在世界系内应用。

### 2.3 仿真（世界系 → 测量点）

`verify_tilt_calibration.py::_simulate_room_scan`：

```python
p_computed = (
    p @ r_tilt @ r_theta @ r_tilt.T
    @ m.T @ r_lidar.T @ m @ r_theta.T
)
```

数学形式：

```
p_measured = p @ R_tilt @ Rz(θ) @ R_tilt.T @ m.T @ R_lidar.T @ m @ Rz(θ).T
```

这是 2.2 的**逆运算**（R_lidar.T 代替 R_lidar，旋转顺序反序），
当标定参数 = 真值时，2.2 的校正应还原出物理点 p。

## 3. 各模块一致性检查

| 模块 | 变换公式 | 是否一致 |
|------|---------|---------|
| `servo_sweep_scan.py`（扫描） | `p @ lidar_tilt.T @ mount.T @ to_world @ Rz(θ).T @ tilt.T` | ✅ 基准 |
| `joint_tilt_calibration.py`（标定） | `p @ Rz(θ) @ m.T @ R_lidar @ m @ R_tilt @ Rz(θ).T @ R_tilt.T` | ✅ 一致 |
| `verify_tilt_calibration.py`（仿真） | `p @ R_tilt @ Rz(θ) @ R_tilt.T @ m.T @ R_lidar.T @ m @ Rz(θ).T` | ✅ 为标定的逆运算 |
| `calibrate_planar.py`（旧实验代码） | `p @ Rz(θ) @ R_tilt @ Rz(θ).T @ R_lidar` | ❌ **不一致** |

### 已发现的问题

**`calibrate_planar.py::apply_correction` 公式错误**：

```python
# 错误：lidar 校正直接作用，缺少 m.T @ ... @ m 包裹
corrected_p1[i] = pt1 @ r_theta @ r_tilt @ r_theta.T @ r_lidar

# 正确：应与 joint_tilt_calibration.py 一致
corrected_p1[i] = pt1 @ r_theta @ m.T @ r_lidar @ m @ r_tilt @ r_theta.T @ r_tilt.T
```

问题：lidar 安装偏差应在**雷达系内**应用（`m.T @ R_lidar @ m`），
旧代码直接在世界系应用 `R_lidar`，且旋转顺序错误。

### 默认 m 验证

`joint_tilt_calibration.py` 的默认 m 硬编码为：

```python
m = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
```

与 side-mount 配置的 `mount.T @ to_world` 一致（已验证 `np.allclose = True`）。
换安装配置时必须显式传入 `m = cfg.mount_matrix().T @ cfg.to_world_matrix()`。

## 4. 雷达坐标系可视化（servo_sweep_scan.py）

雷达坐标系的三个轴方向 = `to_world` 映射 + 舵机旋转：

```python
tw = _INSTALL.to_world_matrix()       # 行=雷达轴在世界系的方向（未旋转）
r_theta = rotation_matrix(args.axis, angle)
x_world = tw[0] @ r_theta.T           # 雷达x → 世界-z（朝下）
y_world = tw[1] @ r_theta.T           # 雷达y → 世界y（朝左）
z_world = tw[2] @ r_theta.T           # 雷达z → 世界x（朝前）
```

用 LineSet 画三条轴：X=红、Y=绿、Z=蓝，起点为雷达光心位置。

**注意**：画雷达坐标轴只用 `to_world`，**不用** `mount`。
`mount` 是扫描数据相位，不影响雷达物理安装方向。

## 5. 约定速查表

- `m = mount_matrix().T @ to_world_matrix()`：棱镜相位 + 安装姿态复合
- `R_lidar`：在雷达系内应用（用 `m.T @ R_lidar @ m` 转到世界系）
- `R_tilt`：在世界系内应用
- `Rz(θ)`：舵机旋转，正方向见 `rotation_matrix("z", θ)`
- 点云用**行向量**（`p @ M`），Open3D 变换矩阵是**列向量**（`T @ v`），注意转置
