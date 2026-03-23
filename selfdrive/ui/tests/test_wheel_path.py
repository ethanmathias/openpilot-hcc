import numpy as np

from openpilot.selfdrive.ui.lib.wheel_path import build_wheel_path_points


def test_zero_curvature_renders_straight_path():
  points = build_wheel_path_points(0.0, max_distance=100.0)

  assert points.shape[1] == 3
  assert np.allclose(points[:, 1], 0.0)
  assert np.isclose(points[0, 0], 0.0)
  assert np.isclose(points[-1, 0], 100.0)


def test_positive_curvature_renders_left_arc():
  points = build_wheel_path_points(0.02, max_distance=50.0)

  assert np.all(np.diff(points[:, 0]) > 0.0)
  assert points[-1, 1] > 0.0


def test_negative_curvature_renders_right_arc():
  points = build_wheel_path_points(-0.02, max_distance=50.0)

  assert np.all(np.diff(points[:, 0]) > 0.0)
  assert points[-1, 1] < 0.0


def test_invalid_curvature_falls_back_to_straight_path():
  points = build_wheel_path_points(float("nan"), max_distance=25.0)

  assert np.allclose(points[:, 1], 0.0)
  assert np.isclose(points[-1, 0], 25.0)
