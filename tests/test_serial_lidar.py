import pytest

from drill_rod_scanner.lidar.serial_lidar import LidarConnectionError, SerialLidar


def test_connect_missing_port_raises():
    lidar = SerialLidar(port="/dev/does-not-exist", baudrate=115200)
    with pytest.raises(LidarConnectionError):
        lidar.connect()


def test_close_without_connect_is_safe():
    lidar = SerialLidar(port="/dev/ttyUSB1", baudrate=115200)
    lidar.close()  # 不应抛异常