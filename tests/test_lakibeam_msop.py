"""LakiBeam MSOP 解析器单元测试（不依赖硬件）。

构造符合协议的手工 UDP 包，验证解析逻辑。
"""

import struct

import numpy as np
import pytest

from scripts.lakibeam_viewer import MSOPParser, ScanPoint, scan_to_xy

# 协议常量
DATA_FLAG = 0xEEFF


def build_msop_packet(
    azimuths_deg: list[float],
    dist_mm: int = 2300,
    rssi: int = 49,
) -> bytes:
    """构造一个 1248 字节的 MSOP 包。

    Args:
        azimuths_deg: 12 个 Data Block 的角度（度）
        dist_mm: 所有点的距离（mm）
        rssi: 所有点的回波强度
    """
    assert len(azimuths_deg) == 12
    body = bytearray()

    for az_deg in azimuths_deg:
        az_raw = int(round(az_deg * 100))  # 0.01° 单位
        block = bytearray()
        block += struct.pack("<H", DATA_FLAG)      # DataFlag
        block += struct.pack("<H", az_raw)          # Azimuth
        for _ in range(16):                          # 16 个测距结果
            block += struct.pack("<HBHB", dist_mm, rssi, 0, 0)
        assert len(block) == 100
        body += block

    body += struct.pack("<I", 0x0EA82087)  # Timestamp
    body += struct.pack("<H", 0)           # Factory

    packet = b"\x00" * 42 + bytes(body)    # 42B UDP/IP 头
    assert len(packet) == 1248
    return packet


def test_packet_size_constant():
    assert MSOPParser.parse_packet(b"\x00" * 100) == []
    full = build_msop_packet([0.0] * 12)
    assert len(full) == 1248


def test_parse_valid_packet_all_points():
    # 12 块 × 16 点 = 192 点，全部有效
    az = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0,
          60.0, 70.0, 80.0, 90.0, 100.0, 110.0]
    packet = build_msop_packet(az)
    pts = MSOPParser.parse_packet(packet)
    assert len(pts) == 192


def test_parse_azimuth_values():
    az = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0,
          60.0, 70.0, 80.0, 90.0, 100.0, 110.0]
    packet = build_msop_packet(az)
    pts = MSOPParser.parse_packet(packet)

    # 块 0 的 16 个点：角度 0.0 ~ 0.625（分辨率 10/16 = 0.625°）
    block0 = pts[:16]
    np.testing.assert_allclose(
        [p.angle for p in block0],
        [i * 10.0 / 16.0 for i in range(16)],
        atol=1e-6,
    )
    # 第 1 块从 10.0 开始
    assert abs(pts[16].angle - 10.0) < 1e-6


def test_parse_skips_zero_distance():
    az = [0.0] * 12
    # 构造：块 0 点 0 距离为 0（无效），其余正常
    body = bytearray()
    for idx in range(12):
        block = bytearray()
        block += struct.pack("<H", DATA_FLAG)
        block += struct.pack("<H", 0)  # azimuth 0
        for i in range(16):
            if idx == 0 and i == 0:
                block += struct.pack("<HBHB", 0, 0, 0, 0)  # 距离 0
            else:
                block += struct.pack("<HBHB", 2300, 49, 0, 0)
        body += block
    body += struct.pack("<I", 0) + struct.pack("<H", 0)
    packet = b"\x00" * 42 + bytes(body)

    pts = MSOPParser.parse_packet(packet)
    assert len(pts) == 192 - 1  # 只有 1 个无效点被跳过


def test_parse_skips_invalid_flag():
    # 构造：第 1 块 DataFlag 无效（0xFFFF），其 16 点应被跳过
    body = bytearray()
    for idx in range(12):
        block = bytearray()
        flag = 0xFFFF if idx == 1 else DATA_FLAG
        block += struct.pack("<H", flag)
        block += struct.pack("<H", 0)
        for _ in range(16):
            block += struct.pack("<HBHB", 2300, 49, 0, 0)
        body += block
    body += struct.pack("<I", 0) + struct.pack("<H", 0)
    packet = b"\x00" * 42 + bytes(body)

    pts = MSOPParser.parse_packet(packet)
    # 无效块整块跳过 -> 192 - 16 = 176
    assert len(pts) == 176


def test_parse_short_packet_returns_empty():
    assert MSOPParser.parse_packet(b"\x00" * 500) == []


def test_scan_to_xy_conversion():
    # 手册示例：0x08FC = 2300mm = 2.3m，角度 0°
    pts = [ScanPoint(angle=0.0, dist_mm=2300, rssi=49)]
    xy = scan_to_xy(pts)
    np.testing.assert_allclose(xy[0], [2.3, 0.0, 0.0], atol=1e-9)

    # 角度 90° -> (0, r, 0)
    pts = [ScanPoint(angle=90.0, dist_mm=1000, rssi=10)]
    xy = scan_to_xy(pts)
    np.testing.assert_allclose(xy[0], [0.0, 1.0, 0.0], atol=1e-9)


def test_scan_to_xy_empty():
    assert scan_to_xy([]).shape == (0, 3)
