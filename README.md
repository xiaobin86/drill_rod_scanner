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
