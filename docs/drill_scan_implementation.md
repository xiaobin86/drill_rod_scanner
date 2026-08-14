# drill_rod_scanner 扫描方法实现原理

> 日期：2026-08-12
> 依据：当前代码（`main` 分支 HEAD `4909e80`），以代码实际实现为准

## 1. 系统概述

**目标**：用舵机带动转盘旋转，LakiBeam 单线雷达横装于转盘上，逐角度采集 2D 点云帧，
旋转拼接为 3D 点云，实现钻头杆等物体的三维扫描定位。

**核心链路**：
```
舵机(串口) ── 转盘旋转角度 θ
雷达(UDP)  ── 每圈 2D 极坐标点云 (angle, dist_mm, rssi)
                    │
                    ▼
极坐标 → 雷达系xyz → 偏心校正 → 世界系 → 绕转盘轴旋转 → 累计3D点云
                    │
                    ▼
         Open3D 实时显示 / 保存 PLY+npy / ROS2 发布(RViz)
```

## 2. 硬件与协议

### 2.1 舵机（转盘驱动）
- 串口协议：`#<ID:3d>P<位置:4d>T<耗时ms>!`，如 `#000P0500T2000!`
- 位置 P 值 500→2500 映射 360°（`servo_pos_to_angle()` 线性映射）
- 默认波特率 115200

### 2.2 LakiBeam 雷达（MSOP 协议）
- 网络连接：以太网 UDP，默认 `192.168.198.2:2368`，本机网卡 `192.168.198.1/24`
- **MSOP 包结构**（`lakibeam_viewer.py`，实测 UDP 载荷 **1206 字节**，无网络头）：
  ```
  12 × Data Block (100B):
      2B DataFlag (0xEEFF) + 2B Azimuth(0.01°) + 16 × 6B 测距结果
  4B Timestamp + 2B Factory
  每个测距结果: 2B Dist_1 + 1B RSSI_1 + 2B Dist_2 + 1B RSSI_2（取最强回波）
  ```
- **角度插值**：块内 16 点角度线性插值
  `resolution = (Azimuth[1] - Azimuth[0]) / 16`（与官方 ROS2 驱动一致，应用于包内所有块）
- **一圈判定**（`receive_scan()`）：当前包首块方位角 < 上一包首块方位角 = 跨过 360° 边界；
  2 秒无回绕则返回已收集数据防挂起

## 3. 坐标系定义（实测确认，核心！）

> 通过实机验证确定，中途多次修改最终回到此模型：

| 坐标系     | 定义                                                      |
| ------- | ------------------------------------------------------- |
| **雷达系** | **x 向前、y 朝上、z 向右**；扫描弧在 x-y 竖直面（0° 指 +x 前方，90° 指 +y 上方） |
| **世界系** | **z 竖直（转盘旋转轴）、x/y 水平**（转盘平面）                            |

**关键结论**（经物理仿真验证）：
- 扫描平面永远是雷达系的 **x-y 平面**，极坐标→xyz 转换**不因横装改变**
- 横装变换在数学上**恒等**（`mount_transform()` 返回原值）——因为扫描点坐标数值不变，
  变化的只是 y 轴语义（出厂横向 → 横装朝上）
- `to_world()` 完成雷达系 → 世界系的轴映射：
  ```
  世界 x = 雷达 x（前）
  世界 y = 雷达 z（右）
  世界 z = 雷达 y（上）
  ```
  实现：`points[:, [0, 2, 1]]`

## 4. 坐标处理链（每帧）

`servo_sweep_scan.py` 主循环中每帧五步：

### ① 极坐标 → 雷达系 xyz（`scan_to_xy()`）
```python
x = dist_mm/1000 * cos(θ)   # 向前
y = dist_mm/1000 * sin(θ)   # 朝上（竖直扫描弧）
z = offset_z_m              # 向右（安装偏移，通常 0）
```
随后按 `--max-range` 过滤水平距离超限的点。

### ② 横装变换（`mount_transform()`，恒等）
保留步骤以便扩展——若实际安装有额外倾斜，在此叠加旋转矩阵。

### ③ 光心偏心校正（关键物理！）
雷达光心**不在转盘旋转轴上**（实测偏 5.5cm），转盘旋转时光心绕轴做**圆弧运动**。
同一世界点在不同转盘角度测得距离不同，必须补偿：

```
世界点 P = Rz(θ) · (雷达系测量 p + 光心偏移 d)
```

**必须"先加 d 再旋转"**（`frame[:,0] += offset_x; frame[:,2] += offset_z`）。
若用"先减再旋转"（`p - d`），每帧补偿方向错误 → 点云随角度漂移成"各圆柱面"。
此问题由 `--offset-x / --offset-z` 参数配置，`tests/test_servo_sweep.py::test_eccentric_offset_arc_reconstruction`
有回归测试。几何图解与推导见 [[drill_scan_eccentricity]]。

### ④ 雷达系 → 世界系（`to_world()`）
轴重排 `(x, y, z) → (x, z, y)`，使世界 z 竖直 = 转盘轴。

### ⑤ 绕世界 z 轴（转盘轴）旋转拼接
```python
angle = servo_pos_to_angle(pos, start, end, angle_start, angle_end)
rotated = rotate_points(frame, axis='z', angle)
```
每帧点云绕转盘轴旋转对应角度，写入累计缓冲 → 竖直扫描弧聚合为 3D 扫描面。

## 5. 扫描编排流程

```
1. 打开舵机串口
2. 归位：发 P{start} 指令 → 等待 --home-wait 秒到位
3. 连接雷达 UDP
4. 创建 Open3D 窗口 + 世界系 z=0 水平网格（--grid）
5. 预分配固定大小点云缓冲（NaN 填充）
6. 循环 [start, end] 每步 step：
   a. 发舵机指令 → 等待 --interval 秒
   b. 雷达采一圈 → 坐标处理五步 → 旋转
   c. 增量写入缓冲 → Open3D 更新
7. 扫描完成 → 可选保存点云（--save-dir）
8. 窗口保持打开可交互查看
```

## 5.1 连续转动模式（--continuous，记录+抽帧）

与步进模式不同，连续模式让转盘**一条命令连续转完**，全程记录雷达所有帧，转完后按位置步长抽帧融合。

```
1. 归位到 start
2. 发一条 #000P{end}T{total_ms}! 命令（total_ms = --total-time × 1000），转盘连续转动
3. 转动期间持续接收雷达帧，每帧记录：
   - rec_ts[]     帧接收时间戳（相对命令发出时刻，秒）
   - rec_frames[] 帧原始点云（ScanPoint 列表）
   → 直到 elapsed >= --total-time 停止
4. 全部帧写入文件：output/frames.npz（ts + frames）
5. 读回文件
6. 按位置步长抽帧，对每个位置 pos：
   frac = (pos - SERVO_POS_MIN) / (SERVO_POS_MAX - SERVO_POS_MIN)   # 500-2500 全程比例
   target_t = frac × total_s                                        # 该位置对应时间点
   idx = pick_frame_index(rec_ts, target_t)                         # 取时间戳最近帧
   angle = servo_pos_to_angle(pos)                                  # 位置 → 角度
   process_frame(scan, angle, ...)                                  # 坐标变换 + 融合
```

**关键实现点**：

- **总耗时**：`--total-time` 直接指定（秒），不再由 interval 推导
- **角度映射**：舵机量程硬编码，`SERVO_POS_MIN=500, SERVO_POS_MAX=2500` →
  `角度 = (pos - 500) / (2500 - 500) × 360`，无需手动指定角度范围
- **抽帧**：`pick_frame_index(timestamps, target_t)` 从记录帧中选时间戳最接近 target_t 的帧
  （`min(range(len(ts)), key=lambda i: abs(ts[i] - target_t))`）
- **坐标变换**：复用 `process_frame()`，与步进模式完全一致（偏心/世界系/旋转）

**举例**（`--start 500 --end 2500 --step 50 --total-time 60`）：

| 位置 pos | 时间点 target_t | 角度 |
|---------|----------------|------|
| 500 | 0s | 0° |
| 550 | 1.5s | 9° |
| 1500 | 30s | 180° |
| 2500 | 60s | 360° |

共抽 41 帧（2000/50 + 1），每帧用位置映射角度融合成 3D 点云。

### 5.2 自动发布（--publish-topic）

扫描抽帧融合完成后，若指定 `--publish-topic`，自动发布点云到 ROS2 topic：

- 复用 `publish_pointcloud.publish()`（内部是 `while rclpy.ok()` 循环）
- 在**后台线程**运行（daemon），不阻塞 Open3D 窗口主循环
- `try/except` 包裹：本机无 ROS2 时打印失败提示，不影响扫描
- 参数：`--publish-topic`（topic 名）、`--publish-frame`（frame_id，默认 map）、
  `--publish-rate`（频率 Hz，默认 2）
- 关键：发布前每帧设置**当前时间戳**（`node.get_clock().now().to_msg()`），
  RViz 依赖 stamp 查询 TF，stamp=0 会导致不显示

**空帧防护**：连续模式若全程未采到任何雷达帧，明确报错
`未采到任何雷达帧，请检查雷达网络/配置` 并中止（避免 `pick_frame_index` 对空列表崩溃）。

**运行环境**：`ros_humble` conda 环境（RoboStack 安装 ROS2 humble + rviz2）。
激活时自动配置（`etc/conda/activate.d/`）：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
`ROS_DOMAIN_ID=0`。使用：
```bash
conda activate ros_humble
python3 scripts/servo_sweep_scan.py --continuous --publish-topic /drill_scan_cloud ...
rviz2   # 另一终端：Fixed Frame=map → Add → PointCloud2 → /drill_scan_cloud
```

## 6. 可视化与性能优化

### 6.1 Open3D 渲染（三个关键坑）
1. **首帧有真实数据后才 add_geometry**：空点云/全 NaN 包围盒退化 → 画面空白
2. **点数不变用 update_geometry 快速路径**：预分配固定大小缓冲（NaN 填充，
   Open3D 渲染时跳过且不参与包围盒），避免每帧 remove/add 重建 GPU 缓冲导致的卡顿
3. 视角：`set_front([0,0,1]) set_up([0,1,0]) set_lookat([0,0,0])` 俯视世界系

### 6.2 网格与坐标轴
- `--grid`：世界系 z=0 水平面灰色网格（半宽默认= max-range）
- `create_world_axes()`：原点三轴标定（X 红 / Y 绿 / Z 蓝），便于判断点云方位

## 7. 数据保存与 ROS2 发布

### 7.1 保存（`--save-dir`）
扫描完成后 `save_cloud()` 保存：
- `cloud.ply`（Open3D 点云，可被 CloudCompare 打开）
- `cloud.npy`（numpy 原始 (n,3) 数组，供 ROS2 发布）

### 7.2 ROS2 发布（`scripts/publish_pointcloud.py`）
读取点云 → 发布 `sensor_msgs/PointCloud2` 到 `/drill_scan_cloud`，RViz 可查看：
```bash
# 本机扫描保存
python scripts/servo_sweep_scan.py --save-dir output ...

# ROS2 环境（Docker pallet_vision:humble）
source /opt/ros/humble/setup.bash
python3 scripts/publish_pointcloud.py --file output/cloud.npy --topic /drill_scan_cloud
```

**两个已踩坑的 API 注意点**：
1. **`sensor_msgs_py.point_cloud2`**（不是 `sensor_msgs.point_cloud2`）——ROS2 humble 已拆分
2. **Header 必须设当前时间戳** `node.get_clock().now().to_msg()`——stamp=0 时 RViz 无法查询 TF，点云不显示

## 8. 命令行参数汇总（servo_sweep_scan.py）

| 参数 | 默认 | 说明 |
|------|------|------|
| `--port` | /dev/ttyUSB0 | 舵机串口 |
| `--start/--end/--step` | 500/1000/10 | 舵机位置 P 范围与步进（500-2500=360°） |
| `--interval` | 2.0 | 每位置停留秒数 |
| `--move-time` | 2000 | 舵机移动耗时 ms |
| `--home-wait` | 3.0 | 归位后等待到位秒数 |
| `--axis` | z | 旋转轴（世界 z = 转盘轴） |
| `--continuous` | - | 连续转动模式（记录全部帧后抽帧融合） |
| `--total-time` | 60.0 | 连续模式转盘总耗时（秒） |
| `--offset-x/--offset-z` | 0 | 光心偏心校正（米） |
| `--max-range` | 50 | 最大显示/拼接距离 |
| `--save-dir` | "" | 保存点云目录（空=不保存） |
| `--grid` | -1 | 网格半宽（<0 自动= max-range，0 关闭） |
| `--grid-step` | 1.0 | 网格线间距 |
| `--dry-run` | - | 只打印舵机指令不连硬件 |

> 角度说明：舵机位置 500-2500 固定映射 0-360°，角度由位置自动计算，无需手动指定。

## 9. 测试覆盖（39 项全部通过）

| 测试文件 | 覆盖内容 |
|----------|---------|
| `test_servo_sweep.py` | 旋转矩阵、to_world 轴映射、角度映射、**偏心圆弧重建回归**、保存点云 |
| `test_lakibeam_msop.py` | MSOP 包解析（角度插值/无效点跳过/短包）、极坐标转 xyz、回环接收集成 |
| `test_stitcher.py` | 拼接算法（旋转合并/体素下采样） |
| `test_scanner.py` | 扫描编排（mock 串口：角度序列/采帧/拼接/空帧跳过） |
| `test_cli.py` | 配置加载 |
| `test_serial_servo/lidar.py` | 驱动接口骨架 |

## 10. 关键实现坑位回顾（调试经验）

1. **MSOP 载荷 1206B 无网络头**：tcpdump 实测 `length 1206`，42B 网络头由内核剥离，
   解析器 `offset=0`
2. **一圈检测用方位角回绕**：`当前角 < 上一角` 判定跨 360°，不依赖 0° 起扫假设
3. **Open3D 空包围盒空白**：全 NaN/空点云 add → 退化包围盒 → 不渲染；首帧真实数据后再 add
4. **增长点云卡顿**：点数变化必须 remove/add 重建 GPU 缓冲；预分配固定缓冲 + update_geometry 解决
5. **偏心先加 d 再旋转**：`P = Rz(θ)·(p + d)`，顺序/符号错 → "各圆柱面"漂移
6. **坐标系以实测为准**：最终采用 x前/y上/z右 + to_world=(x,z,y) + 绕世界 z 旋转，
   与实机点云验证一致
