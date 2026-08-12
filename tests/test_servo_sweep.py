"""servo_sweep_scan 旋转/角度映射单元测试（纯函数，不依赖硬件）。"""

import numpy as np
import pytest

from scripts.servo_sweep_scan import rotation_matrix, rotate_points, servo_pos_to_angle


def test_rotation_matrix_x_90deg():
    # 绕 x 轴转 90°: y->z, z->-y
    m = rotation_matrix("x", 90.0)
    np.testing.assert_allclose(m @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-9)
    np.testing.assert_allclose(m @ np.array([0.0, 0.0, 1.0]), [0.0, -1.0, 0.0], atol=1e-9)


def test_rotation_matrix_y_90deg():
    # 绕 y 轴转 90°: z->x, x->-z
    m = rotation_matrix("y", 90.0)
    np.testing.assert_allclose(m @ np.array([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, -1.0], atol=1e-9)


def test_rotation_matrix_z_90deg():
    # 绕 z 轴转 90°: x->y, y->-x
    m = rotation_matrix("z", 90.0)
    np.testing.assert_allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(m @ np.array([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-9)


def test_rotation_matrix_invalid_axis():
    with pytest.raises(ValueError):
        rotation_matrix("w", 90.0)


def test_rotate_points_shape_and_values():
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    out = rotate_points(pts, "z", 90.0)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out[0], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out[1], [-2.0, 0.0, 0.0], atol=1e-9)


def test_servo_pos_to_angle_mapping():
    # start=500 -> angle_start, end=1000 -> angle_end, 中点映射一半
    assert servo_pos_to_angle(500, 500, 1000, 0.0, 180.0) == 0.0
    assert servo_pos_to_angle(1000, 500, 1000, 0.0, 180.0) == 180.0
    assert servo_pos_to_angle(750, 500, 1000, 0.0, 180.0) == 90.0


def test_servo_pos_to_angle_flat_range():
    # start == end 时返回 angle_start，不除零
    assert servo_pos_to_angle(500, 500, 500, 30.0, 60.0) == 30.0
