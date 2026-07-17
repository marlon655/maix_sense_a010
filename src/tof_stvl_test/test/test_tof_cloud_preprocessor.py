from types import SimpleNamespace

import numpy as np
import pytest

from tof_stvl_test.tof_cloud_preprocessor import (
    select_finite_points_in_range,
    transform_points,
)


def make_transform(x=0.0, y=0.0, z=0.0,
                   qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    ))


def test_range_filter_preserves_points_at_floor_height():
    points = np.array([
        [0.10, 0.0, 0.00],
        [1.00, 0.0, 0.00],
        [3.00, 0.0, 0.00],
        [np.nan, 0.0, 0.50],
    ], dtype=np.float32)

    filtered = select_finite_points_in_range(points, 0.05, 2.0)

    np.testing.assert_allclose(filtered, [
        [0.10, 0.0, 0.00],
        [1.00, 0.0, 0.00],
    ])


def test_invalid_range_is_rejected():
    with pytest.raises(ValueError):
        select_finite_points_in_range([], 1.0, 1.0)


def test_transform_applies_rotation_and_translation():
    root_half = np.sqrt(0.5)
    transform = make_transform(
        x=1.0, y=2.0, z=3.0, qz=root_half, qw=root_half)

    result = transform_points([[1.0, 0.0, 0.0]], transform)

    np.testing.assert_allclose(result, [[1.0, 3.0, 3.0]], atol=1e-6)
