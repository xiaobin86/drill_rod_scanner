"""level_scan 软件调平模块单元测试。"""

import numpy as np
import pytest
import yaml

from scripts.install_config import InstallConfig
from scripts.level_scan import align_to_z_matrix, fit_plane_normal, level_from_cloud


def test_fit_plane_normal_horizontal():
    # 水平面点云 → 法向量应为 (0,0,1)
    rng = np.random.default_rng(42)
    xy = rng.uniform(-1, 1, (500, 2))
    pts = np.column_stack([xy, np.full(500, 0.5)])
    n = fit_plane_normal(pts)
    np.testing.assert_allclose(n, [0, 0, 1.0], atol=1e-9)


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


def test_align_to_z_matrix():
    # 把倾斜法向量对齐到 z，误差应为 0
    n = np.array([0.1, -0.2, 0.975])
    n /= np.linalg.norm(n)
    r = align_to_z_matrix(n)
    aligned = r @ n
    np.testing.assert_allclose(aligned, [0, 0, 1.0], atol=1e-9)
    # 是旋转矩阵（正交 + det=1）
    np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(r), 1.0)


def test_align_to_z_identity_for_z():
    r = align_to_z_matrix(np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(r, np.eye(3), atol=1e-9)


def test_level_from_cloud_tilted_plane_recovers_horizontal():
    # 端到端：倾斜平面 → 校正矩阵 → 平面恢复水平
    rng = np.random.default_rng(1)
    xy = rng.uniform(-2, 2, (1000, 2))
    pts = np.column_stack([xy, np.full(1000, 0.0)])
    th = np.deg2rad(2.5)
    c, s = np.cos(th), np.sin(th)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    tilted = pts @ rx.T

    n_fit, r_align = level_from_cloud(tilted)
    corrected = tilted @ r_align.T
    n_corrected = fit_plane_normal(corrected)
    err = np.degrees(np.arccos(np.clip(n_corrected[2], -1, 1)))
    assert err < 1e-6


def test_level_correction_roundtrip(tmp_path):
    # 写回 YAML 后能重新加载
    cfg = InstallConfig.side_mount()
    path = cfg.save(tmp_path / "install.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    r = np.array([[1.0, 0.0, 0.0], [0.0, 0.999, -0.017], [0.0, 0.017, 0.999]])
    data["turntable_level_correction"] = r.tolist()
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    loaded = InstallConfig.load(path)
    np.testing.assert_allclose(loaded.level_correction, r, atol=1e-9)


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
