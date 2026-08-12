"""舵机串口驱动。

接口签名已定死（connect/set_angle/read_angle/close），
具体的串口协议命令由用户后续补充（见下方 TODO 标记）。
"""

from __future__ import annotations

import serial


class ServoConnectionError(Exception):
    """舵机串口连接失败。"""


class ServoTimeoutError(Exception):
    """舵机响应超时。"""


class SerialServo:
    """通过串口控制舵机角度。"""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        """打开串口。"""
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            raise ServoConnectionError(f"无法连接舵机串口 {self.port}: {e}") from e

        # TODO(用户补充): 舵机串口协议命令 ——
        #  例：发送查询/握手指令确认舵机在线，如 b"\xAA\x55..."。

    def set_angle(self, angle_deg: float) -> None:
        """设置舵机目标角度（度）。"""
        self._require_connected()
        # TODO(用户补充): 协议命令 ——
        #  将 angle_deg 按舵机量程换算为串口指令字节并发送。
        raise NotImplementedError(
            "舵机 set_angle 串口协议命令待用户补充"
        )

    def read_angle(self) -> float:
        """读取舵机当前角度（度）。"""
        self._require_connected()
        # TODO(用户补充): 协议命令 ——
        #  发送查询指令，解析返回的角度字节。
        raise NotImplementedError(
            "舵机 read_angle 串口协议命令待用户补充"
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _require_connected(self) -> None:
        if self._ser is None:
            raise ServoConnectionError("舵机串口未连接，请先调用 connect()")
