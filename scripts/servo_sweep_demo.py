#!/usr/bin/env python
"""舵机位置扫描 demo:每 interval 秒把 #000 号舵机位置 +step,从 start 转到 end.

默认:#000P0500T2000! -> #000P0510T2000! -> ... -> #000P1000T2000!

用法:
  python scripts/servo_sweep_demo.py                     # 默认 500->1000, 每 2s +10
  python scripts/servo_sweep_demo.py --dry-run           # 只打印指令,不连串口
  python scripts/servo_sweep_demo.py --start 100 --end 300 --step 20 --interval 1
"""
import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--start", type=int, default=500, help="起始位置")
    parser.add_argument("--end", type=int, default=1000, help="结束位置(含)")
    parser.add_argument("--step", type=int, default=10, help="每次增量")
    parser.add_argument("--interval", type=float, default=2.0, help="发送间隔秒")
    parser.add_argument("--time", type=int, default=2000, help="T 值,移动耗时 ms")
    parser.add_argument("--dry-run", action="store_true", help="只打印指令不连接串口")
    args = parser.parse_args()

    ser = None
    if not args.dry_run:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"[open] {args.port} @ {args.baud} baud")

    try:
        for pos in range(args.start, args.end + 1, args.step):
            cmd = f"#000P{pos:04d}T{args.time}!"
            print(f"[send] {cmd}")
            if ser:
                ser.write(cmd.encode())
                ser.flush()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[done] Ctrl+C 退出")
    finally:
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
