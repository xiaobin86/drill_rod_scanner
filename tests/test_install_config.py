"""install_config 安装配置模块单元测试。"""

import numpy as np
import pytest

from scripts.install_config import InstallConfig, rotation_matrix


def test_side_mount_default():
    cfg = InstallConfig.side_mount()
    assert cfg.name == "side-mount"
    assert cfg.mount_axis == "z" and cfg.mount_angle_deg == 90.0
    assert (cfg.world_x, cfg.world_y, cfg.world_z) == ("z", "y", "-x")


def test_upright_preset():
    cfg = InstallConfig.upright()
    assert cfg.mount_angle_deg == 0.0
    assert (cfg.world_x, cfg.world_y, cfg.world_z) == ("x", "y", "z")
    # 正装 to_world 恒等
    pts = np.eye(3)
    np.testing.assert_allclose(cfg.to_world(pts), pts)


def test_mount_matrix_rotates_x_to_y():
    # 横装 mount：绕雷达 z 转 90°，x → y
    cfg = InstallConfig.side_mount()
    m = cfg.mount_matrix()
    np.testing.assert_allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(m @ np.array([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-9)
    assert np.isclose(np.linalg.det(m), 1.0)


def test_to_world_matrix_side_mount():
    # 横装 to_world：雷达 x(下)->世界 -z, y(左)->世界 y, z(前)->世界 x
    cfg = InstallConfig.side_mount()
    pts = np.eye(3)
    out = cfg.to_world(pts)
    np.testing.assert_allclose(out[0], [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(out[1], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out[2], [1.0, 0.0, 0.0], atol=1e-9)
    assert np.isclose(np.linalg.det(cfg.to_world_matrix()), 1.0)


def test_save_load_roundtrip(tmp_path):
    cfg = InstallConfig.side_mount()
    path = cfg.save(tmp_path / "install.yaml")
    loaded = InstallConfig.load(path)
    assert loaded == cfg


def test_load_invalid_axis():
    cfg = InstallConfig.load  # noqa: F841
    with pytest.raises(ValueError):
        InstallConfig(world_x="w").to_world_matrix()


def test_rotation_matrix_axes():
    m = rotation_matrix("z", 90.0)
    np.testing.assert_allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
    m = rotation_matrix("y", 90.0)
    np.testing.assert_allclose(m @ np.array([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], atol=1e-9)
    m = rotation_matrix("x", 90.0)
    np.testing.assert_allclose(m @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-9)
    with pytest.raises(ValueError):
        rotation_matrix("w", 90.0)


def test_full_chain_side_mount_single_frame():
    # 完整链：雷达系 0° 指 +x（下），经 mount + to_world 后 0° 应指世界 y（水平）
    cfg = InstallConfig.side_mount()
    p = np.array([1.0, 0.0, 0.0])  # 0° 扫描点，雷达 x（下）
    w = cfg.to_world(cfg.mount_transform(p.reshape(1, 3)))[0]
    np.testing.assert_allclose(w, [0.0, 1.0, 0.0], atol=1e-9)  # 世界 y（水平左）
