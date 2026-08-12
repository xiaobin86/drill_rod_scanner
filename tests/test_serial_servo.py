import pytest

from drill_rod_scanner.servo.serial_servo import SerialServo, ServoConnectionError


def test_connect_missing_port_raises():
    servo = SerialServo(port="/dev/does-not-exist", baudrate=115200)
    with pytest.raises(ServoConnectionError):
        servo.connect()


def test_close_without_connect_is_safe():
    servo = SerialServo(port="/dev/ttyUSB0", baudrate=115200)
    servo.close()  # 不应抛异常
