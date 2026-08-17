"""servo_sweep_scan 旋转/角度映射单元测试（纯函数，不依赖硬件）。"""

import numpy as np
import pytest

from scripts.servo_sweep_scan import (
    mount_transform,
    pick_frame_index,
    rotation_matrix,
    rotate_points,
    save_cloud,
    servo_pos_to_angle,
    to_world,
)


def test_pick_frame_index_nearest():
    # 时间戳列表中找到最接近目标时刻的帧索引
    ts = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert pick_frame_index(ts, 0.0) == 0
    assert pick_frame_index(ts, 1.4) == 1
    assert pick_frame_index(ts, 2.5) == 2   # 2.5 距 2 和 3 等距，min 取先出现的 2
    assert pick_frame_index(ts, 4.9) == 5
    assert pick_frame_index(ts, 100.0) == 5  # 超范围取末尾


def test_pick_frame_index_single():
    assert pick_frame_index([3.5], 100.0) == 0


def test_save_cloud(tmp_path):
    # 保存 (n,3) 点云为 PLY + numpy，验证文件生成与内容
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    files = save_cloud(pts, str(tmp_path))
    assert (tmp_path / "cloud.ply").exists()
    assert (tmp_path / "cloud.npy").exists()
    loaded = np.load(tmp_path / "cloud.npy")
    np.testing.assert_allclose(loaded, pts)


def test_mount_transform_identity():
    # 横装变换数学上恒等（扫描面固定在雷达 x-y 平面，坐标系跟随雷达）
    pts = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 0.0]])
    out = mount_transform(pts)
    np.testing.assert_allclose(out, pts)


def test_to_world_axis_mapping():
    # 横装变换（绕世界 y 轴向下转 90°）: 雷达系 (x下, y左, z前) -> 世界系
    # 世界 x=雷达 z（前）, 世界 y=雷达 y（左）, 世界 z=-雷达 x（下→上）
    pts = np.array([
        [1.0, 0.0, 0.0],   # 雷达 x（下）-> 世界 -z
        [0.0, 1.0, 0.0],   # 雷达 y（左）-> 世界 y
        [0.0, 0.0, 1.0],   # 雷达 z（前）-> 世界 x
    ])
    out = to_world(pts)
    np.testing.assert_allclose(out[0], [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(out[1], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out[2], [1.0, 0.0, 0.0], atol=1e-9)


def test_to_world_then_rotate_z():
    # 雷达前方点 (0,0,1) -> 世界 (1,0,0), 绕世界 z（=转盘轴）转 90° -> (0,1,0)
    # 即单帧弧（世界 y-z 平面）绕转盘轴（世界 z）扫出 3D
    pts = to_world(np.array([[0.0, 0.0, 1.0]]))
    out = rotate_points(pts, "z", 90.0)
    np.testing.assert_allclose(out[0], [0.0, 1.0, 0.0], atol=1e-9)


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
    # 舵机量程固定：500 -> 0°, 2500 -> 360°, 线性映射
    assert servo_pos_to_angle(500) == 0.0
    assert servo_pos_to_angle(2500) == 360.0
    assert servo_pos_to_angle(1500) == 180.0
    assert servo_pos_to_angle(1000) == 90.0


def test_turntable_aggregation_axis_y():
    """转盘绕世界 z（竖直）旋转 = 雷达系 y 轴，验证扫描弧绕 y 轴聚合。

    雷达横装扫描弧在 x-y 竖直平面：前方点 (1,0,0) 绕 y 轴转 90° 后
    应转到 z 方向（水平），y 分量不变——即竖直弧绕竖直轴扫出水平扇面。
    """
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = rotate_points(pts, "y", 90.0)
    # (1,0,0) -> (0,0,-1)：前方点转到 -z（水平方向）
    np.testing.assert_allclose(out[0], [0.0, 0.0, -1.0], atol=1e-9)
    # (0,1,0) 在旋转轴上不变（竖直轴不随转盘旋转）
    np.testing.assert_allclose(out[1], [0.0, 1.0, 0.0], atol=1e-9)


def test_eccentric_offset_arc_reconstruction():
    """光心偏心圆弧运动的完整物理循环：世界点必须在各角度被精确重建。

    物理模型：雷达系 x下/y左/z前，to_world（横装变换：世界x=雷达z、世界y=雷达y、世界z=-雷达x），
    光心偏移 d（雷达系常量），转盘转 θ 时光心世界位置 = Rz(θ)·T·d。
    重建公式（修复后）：P = Rz(θ)·T·(p + d)——先加 d 再旋转。
    这是"各圆柱面" bug 的回归测试：若用 p - d（旧错误实现），
    重建点会随角度漂移而非固定在 P。
    """
    P = np.array([2.0, 0.0, 0.5])   # 世界系墙上的固定点
    d = np.array([0.055, 0.0, 0.0])  # 光心偏移（雷达系 x 下方 5.5cm）
    T = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # to_world（横装变换）

    def rotz(theta: float) -> np.ndarray:
        c, s = np.cos(np.deg2rad(theta)), np.sin(np.deg2rad(theta))
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    for theta in [0, 45, 90, 180, 270]:
        C = rotz(theta) @ (T @ d)                    # 光心世界位置（圆弧运动）
        p_radar = np.linalg.inv(T) @ (rotz(-theta) @ (P - C))  # 雷达系测量
        # 修复后的重建：先加 d（雷达系），再 to_world，再绕世界 z（转盘轴）旋转
        recon = rotate_points(to_world((p_radar + d).reshape(1, 3)), "z", theta)[0]
        np.testing.assert_allclose(recon, P, atol=1e-9)

        # 旧错误实现（减 d）应产生漂移——验证测试能捕获回归
        bad = rotate_points(to_world((p_radar - d).reshape(1, 3)), "z", theta)[0]
        assert not np.allclose(bad, P, atol=1e-6), f"θ={theta}° 旧实现竟重建成功"
