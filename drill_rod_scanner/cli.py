"""CLI 辅助：配置加载与扫描执行。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from drill_rod_scanner.io import save_result
from drill_rod_scanner.lidar.serial_lidar import SerialLidar
from drill_rod_scanner.scanner import Scanner
from drill_rod_scanner.servo.serial_servo import SerialServo


def load_config(path: str | Path) -> dict[str, Any]:
    """从 YAML 文件加载配置。"""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_from_config(cfg: dict[str, Any]) -> None:
    """按配置执行扫描并导出结果。"""
    servo = SerialServo(
        port=cfg["servo"]["port"],
        baudrate=cfg["servo"]["baudrate"],
    )
    lidar = SerialLidar(
        port=cfg["lidar"]["port"],
        baudrate=cfg["lidar"]["baudrate"],
    )
    scan_cfg = cfg["scan"]
    stitch_cfg = cfg.get("stitch", {})
    scanner = Scanner(
        servo=servo,
        lidar=lidar,
        from_deg=scan_cfg["from_deg"],
        to_deg=scan_cfg["to_deg"],
        step_deg=scan_cfg["step_deg"],
        settle_time_s=scan_cfg.get("settle_time_s", 0.5),
        voxel_size=stitch_cfg.get("voxel_size"),
    )
    result = scanner.scan()
    out_cfg = cfg.get("output", {})
    files = save_result(
        result,
        output_dir=out_cfg.get("dir", "output"),
        cloud_format=out_cfg.get("cloud_format", "ply"),
    )
    print(f"扫描完成: {len(result.frames)} 帧, {result.empty_frames} 空帧跳过")
    for kind, path in files.items():
        print(f"  {kind}: {path}")