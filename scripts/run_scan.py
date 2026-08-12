#!/usr/bin/env python3
"""drill_rod_scanner CLI 入口。

用法:
    python scripts/run_scan.py [--config config/scanner.yaml]
"""

from __future__ import annotations

import argparse

from drill_rod_scanner.cli import load_config, run_from_config


def main() -> None:
    parser = argparse.ArgumentParser(description="钻头杆旋转扫描点云采集")
    parser.add_argument(
        "--config", default="config/scanner.yaml",
        help="配置文件路径（默认 config/scanner.yaml）",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    run_from_config(cfg)


if __name__ == "__main__":
    main()