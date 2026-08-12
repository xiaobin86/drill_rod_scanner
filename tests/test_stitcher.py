import numpy as np
import pytest

from drill_rod_scanner.stitching.stitcher import stitch


def test_single_frame_angle_zero_unchanged():
    frame = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.5]])
    out = stitch([frame], [0.0])
    np.testing.assert_allclose(out, frame, atol=1e-9)


def test_single_frame_rotated_90_deg_around_z():
    # 绕 Z 轴旋转 90°：x->y, y->-x
    frame = np.array([[1.0, 0.0, 0.0]])
    out = stitch([frame], [90.0])
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_two_frames_merged():
    f0 = np.array([[1.0, 0.0, 0.0]])
    f90 = np.array([[0.0, 1.0, 0.0]])
    out = stitch([f0, f90], [0.0, 90.0])
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out[0], [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(out[1], [-1.0, 0.0, 0.0], atol=1e-9)


def test_empty_frame_skipped():
    frame = np.empty((0, 3))
    out = stitch([frame], [0.0])
    assert out.shape == (0, 3)


def test_voxel_downsample():
    # 4 个点挤在一个 1cm 体素内，voxel_size=0.02 合并为 1 个点
    frame = np.array([
        [0.0, 0.0, 0.0],
        [0.001, 0.0, 0.0],
        [0.0, 0.001, 0.0],
        [0.0, 0.0, 0.001],
    ])
    out = stitch([frame], [0.0], voxel_size=0.02)
    assert out.shape[0] == 1


def test_angle_list_length_mismatch_raises():
    with pytest.raises(ValueError):
        stitch([np.zeros((1, 3))], [0.0, 10.0])
