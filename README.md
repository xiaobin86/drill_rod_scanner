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

## LakiBeam 雷达实时可视化

连接 LakiBeam 系列（1/1S/1L）以太网雷达，实时查看点云。
（官方 Windows 可视化软件为 RBView/Pointcloud，无 Ubuntu 版；本脚本为 Ubuntu 替代方案）

```bash
# 1. 配置网卡：本机 IP 设为 192.168.198.1（与雷达默认地址 192.168.198.2 同网段）
#    Ubuntu 临时配置（以 enp3s0 为例，用 ip addr 查看你的网卡名）：
#    sudo ip addr add 192.168.198.1/24 dev enp3s0
# 2. 浏览器打开 http://192.168.198.2 确认雷达已启动测距（laser_enable=True）
# 3. 运行查看器（默认端口 2368 与雷达出厂配置一致）
conda activate drill_rod_scanner
python scripts/lakibeam_viewer.py --lidar-ip 192.168.198.2 --port 2368
```

常用参数：`--port`（接收端口，需与雷达配置一致）、`--max-range`（最大距离过滤）、
`--min-rssi`（回波强度过滤）、`--height`（扫描平面高度）。Ctrl+C 退出。

## 舵机旋转扫描 + 点云拼接显示

舵机带动雷达逐步旋转，每个位置采一圈点云，按舵机角度旋转拼接为 3D 点云，
Open3D 黑底绿点实时显示：

```bash
conda activate drill_rod_scanner
python scripts/servo_sweep_scan.py \
  --port /dev/ttyUSB0 --start 500 --end 2500 --step 50 --interval 1.5 \
  --axis z --offset-y -0.055 --offset-z 0.025 \
  --save-dir output
```

参数说明：
- `--start/--end/--step`：舵机位置 P 值范围与增量（对应 `servo_sweep_demo.py`）
- `--interval`：每个位置停留秒数（等雷达采完一圈）
- `--axis`：点云旋转轴（默认 `z`：绕世界 z 竖直转盘轴）
- `--install-config`：雷达安装方式 YAML 配置（默认横装 `configs/install_side_mount.yaml`，
  换安装方式复制该文件修改即可，无需改代码）
- `--offset-y/--offset-z`：光心相对转盘轴心的偏心校正（米，雷达系 y/z 方向）
- `--save-dir`：扫描完成后保存点云到该目录（PLY+PCD+numpy），留空不保存
- `--dry-run`：只打印舵机指令，不连串口（可先验证指令格式）

坐标系约定（横装）：雷达系 x 向下、y 向左、z 向前，自转扫描弧在雷达 x-y 平面
（0° 指 +x 下、90° 指 +y 左）；世界系 z 轴竖直（转盘旋转轴）。
安装姿态由 `InstallConfig`（棱镜 0° 相位 + 雷达系→世界系轴映射）描述，
换安装方式改 YAML 配置文件即可。转盘绕世界 z 轴水平旋转，
把不同时刻的竖直扫描弧聚合成 3D 扫描面。

## 360° 自标定修正转盘轴倾斜 + LiDAR 安装偏航

若机械安装导致转盘轴不完全竖直，或雷达绕自身 z 轴有扭转，点云会出现分层、
重影或地面倾斜。可用 360° 自标定同时估计两类参数：

- **转盘轴倾斜**：`tilt.roll` / `tilt.pitch`
- **LiDAR 安装偏航**：`lidar_tilt.yaw`

```bash
# 1. 先做一次 360° 扫描并保存点云
python scripts/servo_sweep_scan.py \
  --port /dev/ttyUSB0 --start 500 --end 2500 --step 50 --interval 1.5 \
  --install-config configs/install_side_mount.yaml \
  --save-dir output/scan_for_calib

# 2. 用保存的点云标定，结果写回原 YAML
python scripts/calibrate_tilt.py \
  --cloud output/scan_for_calib/cloud.npy \
  --install-config configs/install_side_mount.yaml
```

标定完成后，`configs/install_side_mount.yaml` 会新增可读角度字段：

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

后续 `servo_sweep_scan.py` 使用同一配置文件时，会自动应用上述修正。

### 标定可靠性验证（真值仿真）

可用配置文件作为真值，仿真一次 360° 扫描，再运行标定程序检查误差：

```bash
python scripts/verify_tilt_calibration.py \
  --config configs/install_side_mount.yaml \
  --voxel-size 0.05
```

输出会打印 `tilt` 与 `lidar_tilt` 的真值、估计值和误差。该脚本依赖
`joint_tilt_calibration.calibrate_tilt`，也作为标定算法的回归测试。

### 标定能力边界

- **可标定**：`tilt.roll`、`tilt.pitch`、`lidar_tilt.yaw`。
- **固定为 0**：`lidar_tilt.roll`（等价于全局偏航，单次 360° 扫描不可观）、
  `lidar_tilt.pitch`（为避免与 `tilt.pitch` 耦合，程序固定为 0）。
- 标定原理：360° 扫描点云关于实际转盘轴具有 180° 旋转对称性；
  对侧角度扫描的同名点在校正后应重合。算法先通过地面平整度估计
  `lidar_tilt.yaw`，再通过 180° 对侧点重合度估计 `tilt.roll/pitch`，
  交替迭代优化。

## ROS2 发布（RViz 可视化）

扫描保存点云后，可在 ROS2 环境（Docker `pallet_vision:humble` 或装有 ROS2 的机器）发布为 topic：

```bash
# 本机扫描并保存
python scripts/servo_sweep_scan.py --save-dir output ... --max-range 6

# ROS2 环境（Docker/其他机器）：发布点云
# 需先 source ROS2: source /opt/ros/humble/setup.bash
python scripts/publish_pointcloud.py --file output/cloud.npy --topic /drill_scan_cloud

# 另一个终端启动 RViz，添加 PointCloud2 → 选 /drill_scan_cloud 即可查看
```

## 测试

```bash
conda activate drill_rod_scanner
pytest tests/ -v
```

## 当前状态

- 点云拼接算法已完整实现并通过单测（不依赖硬件）。
- LakiBeam MSOP 协议解析器已实现并通过单测（`tests/test_lakibeam_msop.py`）。
- 舵机串口协议命令待补充：`drill_rod_scanner/servo/serial_servo.py`
  与 `drill_rod_scanner/lidar/serial_lidar.py` 中的 `TODO(用户补充)` 标记处。
