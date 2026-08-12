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
  --port /dev/ttyUSB0 --start 500 --end 1000 --step 10 --interval 2 \
  --axis z --angle-start 0 --angle-end 180
```

参数说明：
- `--start/--end/--step`：舵机位置 P 值范围与增量（对应 `servo_sweep_demo.py`）
- `--interval`：每个位置停留秒数（等雷达采完一圈）
- `--axis`：点云绕哪个轴旋转拼接，**取决于舵机实际安装方向**（x/y/z）
- `--angle-start/--angle-end`：start/end 位置对应的旋转角度（度）
- `--dry-run`：只打印舵机指令，不连串口（可先验证指令格式）

雷达坐标系：x 向前、z 向右、y 朝天（扫描平面为 x-z 水平面）。

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
