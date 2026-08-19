# LakiBeam MSOP 协议规范（资源整理）

> 协议信息来源与文字描述整理，供项目开发参考。
> 更新日期：2026-08-18

## 1. 信息来源（资源清单）

LakiBeam 系列（1/1S/1L）激光雷达的协议由厂商**锐驰智光（Richbeam）**定义，
本项目 `scripts/lakibeam_viewer.py` 的解析实现参考以下官方资源：

| 资源 | 类型 | 地址 |
|------|------|------|
| **LakiBeam 激光雷达用户手册**（中文） | PDF 手册（DFRobot 托管） | `https://ws.dfrobot.com.cn/liyf1BSS3V3XXCsvpYTLNgggBeaE`（含 MSOP 协议完整文字描述，第 6 章） |
| **Lakibeam1_SDK_UserManual_EN**（英文 C++ SDK 手册） | Gitee 仓库 | `https://gitee.com/richbeam/Lakibeam1-SDK-UserManual-EN` |
| **Lakibeam1_ROS2_UserManual_EN**（英文 ROS2 驱动手册） | Gitee 仓库 | `https://gitee.com/richbeam/Lakibeam1-ROS2-UserManual-EN` |
| **Lakibeam_ROS1_Driver**（源码参考） | GitHub 仓库 | `https://github.com/RichbeamTechnology/Lakibeam_ROS1_Driver` |
| **Lakibeam_ROS2_Driver**（源码参考） | GitHub 仓库 | `https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver`（`data_type.h` / `lakibeam1_scan.cpp`） |

> 注：DFRobot 托管的用户手册链接可能失效，建议优先用 Gitee 官方仓库。

## 2. MSOP 协议文字描述（摘自官方用户手册第 6 章）

### 2.1 概述

**MSOP** = Main data Stream Output Protocol（主数据流输出协议）。
- I/O 类型：设备输出，电脑解析。
- 传输：以太网 UDP，默认端口 **2368**。
- 内容：激光测距值、回波反射率、水平旋转角度、时间戳。
- **包长度：1248 字节**（42B UDP Header + 1200B 数据块区间 + 4B 时间戳 + 2B 工厂信息）。

### 2.2 MSOP 包结构

```
┌────────────┬──────────────┬──────────┬─────────┐
│ UDP Header │ 12×Data Block│ Timestamp│ Factory │
│  42 Bytes  │  1200 Bytes  │ 4 Bytes  │ 2 Bytes │
└────────────┴──────────────┴──────────┴─────────┘
```

- **UDP Header（42B）**：网络层头，本机 recv 时被内核剥离，**不进入应用层载荷**。
- **Data Block（12 × 100B = 1200B）**：测量数据主体。
- **Timestamp（4B）**：32bit 无符号整型，系统时间，分辨率 **1us**。
- **Factory（2B）**：工厂信息（LakiBeam1/1L 为 `0x37 0x40`）。

### 2.3 Data Block 内部结构（100B）

```
┌──────────┬──────────┬────────────────────────────────────────┐
│ DataFlag │ Azimuth  │ 16 × Measuring Result（每个 6B）        │
│ 2 Bytes  │ 2 Bytes  │                                        │
│  0xFFEE  │ 0.01°/单位│                                        │
└──────────┴──────────┴────────────────────────────────────────┘
```

- **DataFlag（2B）**：标志位 `0xFFEE` 表示有效块。
- **Azimuth（2B）**：水平旋转角度，单位 **0.01°**（uint16）。
- **16 个 Measuring Result**：每个 6B。

### 2.4 Measuring Result（6B）

```
┌──────────────┬─────────┬──────────────┬─────────┐
│ Dist_1 (2B)  │RSSI_1(1B)│ Dist_2 (2B)  │RSSI_2(1B)│
│ 最强回波距离  │最强回波  │ 最后回波距离  │最后回波  │
│ 单位 mm      │ 强度    │ 单位 mm      │ 强度    │
└──────────────┴─────────┴──────────────┴─────────┘
```

- 前 3 字节：**最强回波（Strongest Return）** —— 本项目采用。
- 后 3 字节：**最后回波（Last Return）** —— 本项目忽略。
- 距离单位：**mm**。

### 2.5 块内角度插值（关键算法）

官方手册原文：
> 在每个 Block 中，每一个测距数据输出的水平角度值的计算方法为：
> 首先算出两个相邻 Block 的角度值之差 α1，则每个数据块中相邻两个测距数据的
> 角度递增值为 **α1 / 16**，那么当前 Block 第 N 个（0~15）测距数据的角度值
> Azimuth_N 即可相应计算出来：
>
> `Azimuth_N = Azimuth(当前块) + (α1 / 16) × N`

实现（`lakibeam_viewer.py`，与官方 ROS2 驱动一致）：
```python
resolution = (Azimuth[1] - Azimuth[0]) / 16   # 相邻块角度差 / 16
angle = Azimuth[block] + resolution * i        # 块内第 i 个点 (i=0..15)
```

### 2.6 字节序与无效数据

- **字节序**：小端序（低位在前，高位在后）。官方手册原文：
  > 在进行 Block 角度值、距离信息以及时间戳数据的解算时，MSOP 包中数据均是
  > **高位在后，低位在前**（即小端序）。
- **无效数据**：`DataFlag != 0xFFEE` 或 `dist == 0` 时跳过该点。

### 2.7 一圈末尾的特殊包（重要！）

官方手册原文：
> 在雷达每一圈输出的 MSOP 数据包中，**最末尾的 MSOP 数据包与其它包内容并不
> 完全相同**。以 20/25/30Hz 为例，当一圈数据（1440 组）输出完毕时，末尾包中
> 还剩 96 组数据未能填充，即最末尾的 6 个 Data Block 中的数据，因此自
> **Data Block 6 到 Data Block 11** 中的所有数据均为无效数据，
> 标志位及 Azimuth 均为 **0xFFFF**，所有无效测距数据也均为 **0xFFFF**。

本项目 `receive_scan()` 的一圈判定不依赖 0° 起扫假设，而是：
```python
当前包首块方位角 < 上一包首块方位角  →  跨过 360° 边界，一圈完成
```

## 3. 本项目实现对照（lakibeam_viewer.py）

| 协议要素 | 官方定义 | 本项目实现 |
|---------|---------|-----------|
| UDP 载荷长度 | 1206B（无网络头） | `MSOP_PACKET_SIZE = 12×100 + 4 + 2 = 1206` |
| DataFlag | 0xFFEE | `DATA_FLAG = 0xEEFF`（小端存储） |
| 无效距离 | dist == 0 | `INVALID_DIST = 0`，跳过 |
| 角度插值 | α1/16 | `resolution = (azimuths[1]-azimuths[0])/16` |
| 回波选择 | 最强回波 | 取 `Dist_1 + RSSI_1` |
| 距离单位 | mm | `dist_mm / 1000.0` → m |

## 4. 相关测试

- `tests/test_lakibeam_msop.py`：手工构造 1206B MSOP 包，验证：
  - 角度插值正确性
  - 无效点（dist=0 / flag≠0xFFEE）跳过
  - 短包处理
  - 极坐标→xyz 转换
  - UDP 回环接收（无硬件）

## 5. 已知边界情况

- **末尾包 6 个无效 Block**：标志位 0xFFFF 会被 `flag != DATA_FLAG` 过滤，
  不产生误点；但解析时仍会遍历（性能影响极小）。
- **42B 网络头**：recv 直接得到应用层载荷，无需手动剥离；
  tcpdump 抓包看到的是 1248B（含网络头），应用层是 1206B。

## 6. 时间戳与"包"概念的深入解释

### 6.1 时间戳（Timestamp）—— 雷达内部计时，非 UTC

- **32bit 无符号整型，分辨率 1us，小端序**。
- **起点**：官方手册示例 `0x0EA82087 = 245899399us ≈ 245.9 秒 ≈ 4.1 分钟`。
  数值远小于 Unix 时间戳（≈17 亿秒），因此**不是 UTC/1970 起点**，
  而是**雷达上电后的内部计时器**（从 0 开始计数）。
- **回绕**：32bit 满值 4294967295us ≈ 4295 秒 ≈ **71.6 分钟** 后回绕到 0。
  长时间运行的设备无法凭时间戳本身判断绝对时刻。
- **与其他设备对齐**：LakiBeam 无 GPS/PTP 外部时间同步接口（至少 MSOP 协议
  层面没有）。对齐方式只能是**接收侧打本地墙钟**：
  ```
  point_time_local ≈ packet_receive_time_local − 网络延迟
  ```
  本项目用 `time.monotonic()` 记录接收时刻做一圈判定，未使用雷达时间戳对齐。
- **多雷达场景**：若需对齐两台雷达，需以接收侧本地时间为基准，或引入外部
  同步（PPS/NTP），不能直接比较两台雷达各自的时间戳。

### 6.2 "包"的概念 —— 反射镜扫描的打包传输

**LakiBeam 是单线雷达**，内部有一面**旋转的反射镜**（电机带动），
反射镜转一圈即完成 360° 扫描（不是整机机械旋转）。

```
反射镜旋转一圈 = 360° 扫描 = 1440 组测距数据（20/25/30Hz 时）
        │
        ├── 16 组封装成 1 个 Data Block（块内共用基准角 Azimuth，线性插值）
        │        （16 个点 × 6B = 96B + 4B 头 = 100B）
        │
        ├── 12 个 Data Block 合成 1 个 MSOP UDP 包（12 × 100B = 1200B 数据区）
        │        （= 192 个测距点/包）
        │
        └── 一圈 1440 点 = 7 个整包（1344 点）+ 1 个尾包（96 点）
               尾包只有前 6 个 Block 有效，后 6 个 Block 填 0xFFFF
```

**为什么 16 个测距点一个 Block？**

这是雷达固件/硬件设计决定的**打包粒度**：
- 反射镜连续旋转，角度编码器给出每个采样点的 Azimuth；
- 16 个连续采样点共用 1 个 Azimuth 基准值，块内按
  `α1/16` 线性插值补出每点角度（α1 = 相邻 Block 的 Azimuth 差）；
- 这样 12 × 16 = 192 点/包，1440 点/圈，包数固定可预测，便于接收端重组。

**一句话**：一个 MSOP 包 = 反射镜扫过 **12 个角度格** 的测量结果，
每格（Data Block）含 **16 个采样点**（角度靠插值细分）。
