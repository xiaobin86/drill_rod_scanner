#!/usr/bin/env python
"""串口收发小工具：python scripts/serial_tool.py <send|read|monitor> [args]

用法示例：
  发送文本          python scripts/serial_tool.py send "hello"
  发送十六进制字节  python scripts/serial_tool.py send --hex "55 AA 01 02"
  读取 N 字节       python scripts/serial_tool.py read 64
  持续监听          python scripts/serial_tool.py monitor
  交互模式          python scripts/serial_tool.py interactive

通用参数：--port /dev/ttyUSB0  --baud 115200  --timeout 0.1
"""
import argparse
import time

import serial


def open_port(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"[open] {args.port} @ {args.baud} baud")
    return ser


def fmt_data(data: bytes, as_text: bool) -> str:
    if not as_text:
        return data.hex(" ")
    out = []
    for b in data:
        if 32 <= b < 127:
            out.append(chr(b))
        elif b == 13:
            out.append("\\r")
        elif b == 10:
            out.append("\\n")
        else:
            out.append(f"\\x{b:02x}")
    return "".join(out)


def parse_bytes(data: str) -> bytes:
    parts = data.replace(",", " ").split()
    if len(parts) == 1 and all(c in "0123456789abcdefABCDEF" for c in parts[0]):
        # 纯十六进制串(如 55aa0102)按字节解析
        s = parts[0]
        if len(s) % 2 != 0:
            raise ValueError("hex 串长度必须为偶数")
        return bytes.fromhex(s)
    return bytes(int(p, 16) for p in parts)


def cmd_send(args):
    ser = open_port(args)
    if args.hex:
        payload = parse_bytes(args.data)
    else:
        payload = args.data.encode() + b"\r\n"
    ser.write(payload)
    ser.flush()
    print(f"[send] {fmt_data(payload, args.text)} ({len(payload)} bytes)")
    if args.expect:
        echo = ser.read(args.expect)
        print(f"[recv] {fmt_data(echo, args.text)} ({len(echo)} bytes)")
    elif args.settle:
        # 两段超时:先用较长 timeout 等首字节(设备需要反应时间),
        # 再改用 50ms 快速轮询,连续静默 settle 秒视为响应结束
        chunks = []
        ser.timeout = args.timeout
        first = ser.read(1)
        if not first:
            print("[recv] (无响应,超时未收到任何数据)")
            return
        chunks.append(first)
        ser.timeout = 0.05
        last = time.monotonic()
        while True:
            chunk = ser.read(256)
            now = time.monotonic()
            if chunk:
                chunks.append(chunk)
                last = now
            elif now - last > args.settle:
                break
        total = b"".join(chunks)
        print(f"[recv] {fmt_data(total, args.text)} ({len(total)} bytes)")
        print(f"[done] 收完:静默 {args.settle}s 无新数据")


def cmd_read(args):
    ser = open_port(args)
    data = ser.read(args.n)
    print(f"[recv] {len(data)} bytes: {fmt_data(data, args.text)}")


def cmd_monitor(args):
    ser = open_port(args)
    print("[monitor] Ctrl+C 退出")
    try:
        while True:
            chunk = ser.read(64)
            if chunk:
                print(f"[recv] {fmt_data(chunk, args.text)}")
    except KeyboardInterrupt:
        pass


def cmd_interactive(args):
    ser = open_port(args)
    print("[interactive] 输入即发送(默认按 hex 解析, 文本加前缀 't:'), Ctrl+C 退出")
    try:
        while True:
            line = input(">> ").strip()
            if not line:
                continue
            if line.startswith("t:"):
                ser.write(line[2:].encode() + b"\r\n")
            else:
                ser.write(parse_bytes(line))
            ser.flush()
    except (KeyboardInterrupt, EOFError):
        pass


def main():
    # 子命令解析器不带默认值(SUPPRESS),避免覆盖主解析器已解析的值
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default=argparse.SUPPRESS)
    common.add_argument("--baud", type=int, default=argparse.SUPPRESS)
    common.add_argument("--timeout", type=float, default=argparse.SUPPRESS)
    common.add_argument("--text", action="store_true", default=argparse.SUPPRESS,
                        help="接收数据显示为可读字符(非 --text 时显示 hex)")

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--text", action="store_true",
                        help="接收数据显示为可读字符(非 --text 时显示 hex)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_send = sub.add_parser("send", parents=[common], help="发送数据")
    p_send.add_argument("data", help="文本或 hex(如 '55 AA' / '55aa0102')")
    p_send.add_argument("--hex", action="store_true", help="按 hex 解析")
    p_send.add_argument("--expect", type=int, default=0, help="发送后等待读取的固定字节数")
    p_send.add_argument("--settle", type=float, default=0.0,
                        help="发送后持续读,静默该秒数视为响应结束(响应长度未知时用)")
    p_send.set_defaults(func=cmd_send)

    p_read = sub.add_parser("read", parents=[common], help="读取 N 字节")
    p_read.add_argument("n", type=int, help="读取字节数")
    p_read.set_defaults(func=cmd_read)

    p_monitor = sub.add_parser("monitor", parents=[common], help="持续监听")
    p_monitor.set_defaults(func=cmd_monitor)
    p_interactive = sub.add_parser("interactive", parents=[common], help="交互式收发")
    p_interactive.set_defaults(func=cmd_interactive)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
