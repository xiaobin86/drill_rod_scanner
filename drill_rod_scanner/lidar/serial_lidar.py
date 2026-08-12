"""雷达串口驱动。

接口签名已定死（connect/get_frame/close），
具体的串口协议命令与点云帧解析由用户后续补充（见下方 TODO 标记）。
"""

from __future__ import annotations

import serial
import numpy as np


class LidarConnectionError(Exception):
    """雷达串口连接失败。"""


class LidarTimeoutError(Exception):
    """雷达响应超时。"""


class SerialLidar:
    """通过串口采集雷达点云帧。"""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None

    def connect(self) -> None:
        """打开串口。"""
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            raise LidarConnectionError(f"无法连接雷达串口 {self.port}: {e}") from e

        # TODO(用户补充): 雷达串口协议命令 ——
        #  例：发送启动扫描指令，如 b"\xA5\x20..."。

    def get_frame(self) -> np.ndarray:
        """采集一帧点云，返回形状 (n, 3) 的 numpy 数组（单位：米）。

        返回的坐标系为雷达自身坐标系（扫描平面内 x-y 为水平面，z 为高度）。
        """
        self._require_connected()
        # TODO(用户补充): 协议命令与帧解析 ——
        #  发送单帧采集指令，读取并解析距离/角度数据，
        #  转换为 (n, 3) 直角坐标点云。
        raise NotImplementedError(
            "雷达 get_frame 串口协议命令待用户补充"
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _require_connected(self) -> None:
        if self._ser is None:
            raise LidarConnectionError("雷达串口未连接，请先调用 connect()")
