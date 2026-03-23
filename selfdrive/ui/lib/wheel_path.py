import numpy as np

WHEEL_PATH_POINT_COUNT = 33
WHEEL_PATH_CURVATURE_EPS = 1e-4


def build_wheel_path_points(curvature: float | None,
                            max_distance: float,
                            num_points: int = WHEEL_PATH_POINT_COUNT,
                            z_offset: float = 0.0) -> np.ndarray:
  curvature_value = float(curvature) if curvature is not None else 0.0
  if not np.isfinite(curvature_value):
    curvature_value = 0.0

  arc_lengths = np.linspace(0.0, float(max_distance), int(max(2, num_points)), dtype=np.float32)

  if abs(curvature_value) < WHEEL_PATH_CURVATURE_EPS:
    x = arc_lengths
    y = np.zeros_like(arc_lengths)
  else:
    x = np.sin(curvature_value * arc_lengths) / curvature_value
    y = (1.0 - np.cos(curvature_value * arc_lengths)) / curvature_value

  z = np.full_like(arc_lengths, float(z_offset))
  return np.column_stack((x, y, z)).astype(np.float32)
