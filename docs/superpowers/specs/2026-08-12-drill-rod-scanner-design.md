# drill_rod_scanner 设计文档

> 日期：2026-08-12
> 状态：已获用户批准（技术栈 / 命名 / 目录结构三项确认）

## 1. 项目定位

**drill_rod_scanner** —— 钻头杆点云扫描定位程序。

通过舵机带动激光雷达绕竖直轴旋转，从角度 A 扫到角度 B，按固定角度步进逐帧采集雷达点云，将各帧点云按对应舵机角度做旋转变换后拼接为完整点云，用于钻头杆的定位与测量。

**工作方式**：独立 Python 程序，串口直连舵机与雷达，无需 ROS。

## 2. 技术栈

- Python 3（conda 环境，参照 `jaka_zu35_mujoco_rl` 的 `mujoco` 环境模式）
- `pyserial` —— 舵机 / 雷达串口通信
- `numpy` —— 点云数据表示与旋转矩阵运算
- `open3d` —— 点云 IO（PCD/PLY）、下采样、可视化
- `pyyaml` —— 配置文件解析

## 3. 总体架构

```
硬件层           驱动层              编排层                输出
舵机 ──串口──> SerialServo ──┐
                             ├──> Scanner ──> Stitcher ──> 完整点云
雷达 ──串口──> SerialLidar ──┘        │           │
                        角度步进循环    按角度旋转变换拼接
```

## 4. 目录结构（方案 A：分层模块化）

```
drill_rod_scanner/
├── pyproject.toml            # 包元数据 + 依赖 (pyserial/numpy/open3d/pyyaml)
├── README.md
├── config/
│   └── scanner.yaml          # 舵机串口/角度范围/步进、雷达串口、输出路径
├── drill_rod_scanner/
│   ├── __init__.py
│   ├── servo/
│   │   ├── __init__.py
│   │   └── serial_servo.py   # 舵机驱动接口（协议命令留 TODO）
│   ├── lidar/
│   │   ├── __init__.py
│   │   └── serial_lidar.py   # 雷达驱动接口（协议命令留 TODO）
│   ├── stitching/
│   │   ├── __init__.py
│   │   └── stitcher.py       # 按舵机角度旋转变换 → 拼接完整点云
│   └── scanner.py            # 编排器：A→B 步进 → 采帧 → 拼接 → 导出
├── scripts/
│   └── run_scan.py           # CLI 入口
├── tests/
│   ├── test_stitcher.py      # 拼接正确性单测（不依赖硬件）
│   └── test_scanner.py       # mock 串口跑完整扫描流程
└── docs/
    └── superpowers/specs/    # 本设计文档所在
```

## 5. 组件职责

| 模块 | 职责 | 骨架阶段范围 |
|------|------|------|
| `servo/serial_servo.py` | `connect()` / `set_angle(deg)` / `read_angle()` / `close()`；串口读写 + 协议命令 | 接口签名定死；协议命令留 TODO 占位 |
| `lidar/serial_lidar.py` | `connect()` / `get_frame() -> (n,3) np.ndarray` / `close()`；串口读写 + 协议命令 | 同上 |
| `stitching/stitcher.py` | `stitch(frames, angles)`：每帧点云绕 Z 轴旋转对应角度后合并，可选体素下采样 | 核心算法**本次完整实现**，可单测 |
| `scanner.py` | `scan(from_deg, to_deg, step_deg)`：逐角度 → 舵机到位 → 雷达采帧 → 收集 (angle, points) → 交给 stitcher → 导出 | 编排逻辑完整实现；串口操作走 mock |
| `config/scanner.yaml` | 运行配置集中管理 | 配置文件即可改 |

## 6. 数据流

1. 读取 `config/scanner.yaml`（舵机/雷达串口参数、角度范围、步进、输出路径）。
2. 舵机转到起始角度 A。
3. 循环：当前角度 → 舵机到位（等待稳定）→ 雷达采一帧（n,3）点云 → 记录 `(angle, points)` → 角度 += 步进，直到 B。
4. 将全部帧交给 `Stitcher`：每帧点云绕 Z 轴旋转 `angle` 弧度后合并为一张完整点云。
5. 导出：PCD/PLY 文件 + 原始帧 numpy 文件。

## 7. 关键设计决策

1. **驱动接口面向协议未知**：舵机/雷达具体串口命令用户后续补充。骨架阶段只定死接口签名（`connect`/`set_angle`/`get_frame`/`close`），内部命令以 TODO 标记占位，保证编排与拼接代码不依赖具体硬件命令。
2. **拼接变换可单测**：`stitcher` 无任何串口依赖，喂入已知帧 + 角度即可验证输出，不接硬件也能测。
3. **配置集中**：所有运行参数（串口、角度、输出）走 `scanner.yaml`，硬件细节变化不改代码。
4. **依赖最小化**：仅 pyserial / numpy / open3d / pyyaml。

## 8. 错误处理（骨架级）

- 串口连接失败 → 明确报错信息（串口路径、错误原因）。
- 舵机到位超时 / 雷达无响应 → 抛时序异常 `ScanTimeoutError`，本轮扫描中断并保留已采帧，可恢复续扫。
- 空帧（n=0 或无有效点）→ 跳过并计数，结束时报告。

## 9. 测试

- `tests/test_stitcher.py`：真实数字验证——两帧已知点云按已知角度旋转拼接，断言拼接结果坐标正确（不依赖硬件）。
- `tests/test_scanner.py`：mock 串口（假舵机/假雷达）跑完整扫描流程，断言角度序列、帧采集数量、拼接输出与导出文件均正确。

## 10. 后续填充计划（用户待提供）

- 舵机串口协议命令（波特率、指令格式、角度换算）。
- 雷达串口协议命令（波特率、指令格式、点云帧解析）。
- 雷达坐标系的安装朝向（影响旋转轴对齐）。