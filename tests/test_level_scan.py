"""level_scan 软件调平模块单元测试。"""

import numpy as np
import pytest
import yaml

from scripts.install_config import InstallConfig
from scripts.level_scan import (
    fit_plane_normal,
    level_from_cloud,
    normal_to_tilt_angles,
)


def test_fit_plane_normal_horizontal():
    # 水平面点云 → 法向量应为 (0,0,1)
    rng = np.random.default_rng(42)
    xy = rng.uniform(-1, 1, (500, 2))
    pts = np.column_stack([xy, np.full(500, 0.5)])
    n = fit_plane_normal(pts)
    np.testing.assert_allclose(n, [0, 0, 1.0], atol=1e-9)


def test_fit_plane_normal_large_cloud_downsampled():
    # 百万级点云自动降采样后仍能正确拟合（回归：SVD 内存爆炸）
    rng = np.random.default_rng(3)
    n_pts = 300_000
    xy = rng.uniform(-5, 5, (n_pts, 2))
    pts = np.column_stack([xy, rng.normal(0, 0.001, n_pts)])
    n = fit_plane_normal(pts)
    np.testing.assert_allclose(n, [0, 0, 1.0], atol=1e-6)


def test_fit_plane_normal_tilted():
    # 倾斜平面（绕 x 转 3°）：法向量应含对应分量
    rng = np.random.default_rng(7)
    xy = rng.uniform(-1, 1, (500, 2))
    pts = np.column_stack([xy, np.full(500, 0.0)])
    th = np.deg2rad(3.0)
    c, s = np.cos(th), np.sin(th)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    tilted = pts @ rx.T
    n = fit_plane_normal(tilted)
    # 法向量绕 x 反向转 3°，y 分量应为 0
    np.testing.assert_allclose(n[0], 0.0, atol=1e-6)
    assert n[2] > 0.99


def test_level_from_cloud_tilted_plane_recovers_horizontal():
    # 端到端：倾斜平面 → 倾斜角 → 校正矩阵 → 平面恢复水平
    rng = np.random.default_rng(1)
    xy = rng.uniform(-2, 2, (1000, 2))
    pts = np.column_stack([xy, np.full(1000, 0.0)])
    th = np.deg2rad(2.5)
    c, s = np.cos(th), np.sin(th)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    tilted = pts @ rx.T

    n_fit, tilt_x, tilt_y = level_from_cloud(tilted)
    assert abs(tilt_x) > 0.1  # 绕 x 轴有约 2.5° 倾斜
    assert abs(tilt_y) < 1e-6  # 绕 y 轴无倾斜

    from scripts.install_config import rotation_matrix
    r_align = rotation_matrix("y", tilt_y) @ rotation_matrix("x", tilt_x)
    corrected = tilted @ r_align.T
    n_corrected = fit_plane_normal(corrected)
    err = np.degrees(np.arccos(np.clip(n_corrected[2], -1, 1)))
    assert err < 1e-6


def test_normal_to_tilt_angles_roundtrip():
    # 法向量 → 倾斜角 → 重建矩阵 → 法向量回到 z
    n = np.array([0.02, -0.03, 0.999])
    n /= np.linalg.norm(n)
    tilt_x, tilt_y = normal_to_tilt_angles(n)
    from scripts.install_config import rotation_matrix
    r = rotation_matrix("y", tilt_y) @ rotation_matrix("x", tilt_x)
    np.testing.assert_allclose(r @ n, [0, 0, 1.0], atol=1e-9)


def test_level_correction_roundtrip(tmp_path):
    # 写回可读角度后能重新加载，且 level_correction_matrix 正确
    cfg = InstallConfig.side_mount()
    cfg.level_tilt_x_deg = -1.5
    cfg.level_tilt_y_deg = 2.0
    path = cfg.save(tmp_path / "install.yaml")
    loaded = InstallConfig.load(path)
    assert loaded.level_tilt_x_deg == -1.5
    assert loaded.level_tilt_y_deg == 2.0
    m = loaded.level_correction_matrix()
    from scripts.install_config import rotation_matrix
    expected = rotation_matrix("y", 2.0) @ rotation_matrix("x", -1.5)
    np.testing.assert_allclose(m, expected, atol=1e-9)


def test_rotation_axis_vector_prefers_vector():
    cfg = InstallConfig.side_mount()
    cfg.turntable_axis_vector = np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(cfg.rotation_axis_vector(), [0, 1, 0])
    cfg2 = InstallConfig.side_mount()
    np.testing.assert_allclose(cfg2.rotation_axis_vector(), [0, 0, 1])


def test_invalid_axis_vector():
    cfg = InstallConfig.side_mount()
    cfg.turntable_axis_vector = np.array([0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        cfg.rotation_axis_vector()


def test_rotate_points_with_vector():
    # 绕任意向量旋转：绕 z 向量应与绕 'z' 字符串一致
    from scripts.servo_sweep_scan import rotate_points
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    v = np.array([0.0, 0.0, 1.0])
    out_v = rotate_points(pts, v, 90.0)
    out_s = rotate_points(pts, "z", 90.0)
    np.testing.assert_allclose(out_v, out_s, atol=1e-9)
