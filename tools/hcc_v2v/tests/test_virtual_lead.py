import pytest

from tools.hcc_v2v.virtual_lead import MPH_TO_MPS, load_speed_profile, resolve_speed_scale, sample_profile


def _write_csv(tmp_path, rows):
  path = tmp_path / "scenarios.csv"
  path.write_text("\n".join(",".join(str(c) for c in row) for row in rows) + "\n")
  return path


def test_load_speed_profile_basic(tmp_path):
  path = _write_csv(tmp_path, [
    [0.0, 1.0, 5.0],
    [1.0, 2.0, 6.0],
    [2.0, 3.0, 7.0],
  ])
  times, speeds = load_speed_profile(path, 2)
  assert times == [0.0, 1.0, 2.0]
  assert speeds == [5.0, 6.0, 7.0]


def test_load_speed_profile_stops_at_blank_cell(tmp_path):
  path = _write_csv(tmp_path, [
    [0.0, 5.0],
    [1.0, 6.0],
    [2.0, ""],
    [3.0, 9.0],
  ])
  times, speeds = load_speed_profile(path, 1)
  assert times == [0.0, 1.0]
  assert speeds == [5.0, 6.0]


def test_load_speed_profile_clamps_negative_speed(tmp_path):
  path = _write_csv(tmp_path, [
    [0.0, -1.0],
    [1.0, 4.0],
  ])
  _, speeds = load_speed_profile(path, 1)
  assert speeds == [0.0, 4.0]


def test_load_speed_profile_rejects_bad_index(tmp_path):
  path = _write_csv(tmp_path, [[0.0, 1.0], [1.0, 2.0]])
  with pytest.raises(ValueError):
    load_speed_profile(path, 0)


def test_load_speed_profile_requires_two_samples(tmp_path):
  path = _write_csv(tmp_path, [[0.0, 1.0]])
  with pytest.raises(RuntimeError):
    load_speed_profile(path, 1)


def test_sample_profile_interpolates():
  times = [0.0, 2.0, 4.0]
  speeds = [0.0, 4.0, 4.0]
  v, a = sample_profile(times, speeds, 1.0)
  assert v == pytest.approx(2.0)
  assert a == pytest.approx(2.0)
  v, a = sample_profile(times, speeds, 3.0)
  assert v == pytest.approx(4.0)
  assert a == pytest.approx(0.0)


def test_sample_profile_clamps_to_endpoints():
  times = [1.0, 2.0]
  speeds = [3.0, 5.0]
  assert sample_profile(times, speeds, 0.0) == (3.0, 0.0)
  assert sample_profile(times, speeds, 99.0) == (5.0, 0.0)


def test_resolve_speed_scale_caps_peak_to_mph():
  speeds = [0.0, 7.0, 14.0]
  scale = resolve_speed_scale(speeds, 1.0, 20.0)
  assert max(v * scale for v in speeds) == pytest.approx(20.0 * MPH_TO_MPS)


def test_resolve_speed_scale_never_scales_up():
  speeds = [0.0, 4.0]  # peak ~8.9 mph, well under the cap
  assert resolve_speed_scale(speeds, 1.0, 20.0) == 1.0


def test_resolve_speed_scale_composes_with_explicit_scale():
  speeds = [0.0, 14.0]
  scale = resolve_speed_scale(speeds, 0.9, 20.0)
  assert max(v * scale for v in speeds) == pytest.approx(20.0 * MPH_TO_MPS)
  assert resolve_speed_scale(speeds, 0.5, None) == pytest.approx(0.5)


def test_resolve_speed_scale_rejects_nonpositive():
  speeds = [0.0, 10.0]
  with pytest.raises(ValueError):
    resolve_speed_scale(speeds, 0.0, None)
  with pytest.raises(ValueError):
    resolve_speed_scale(speeds, -1.0, None)
  with pytest.raises(ValueError):
    resolve_speed_scale(speeds, 1.0, 0.0)
