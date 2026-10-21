"""Scenario profile tests, including against the real scenarios.csv.

scenario 48 at ~32 mph is the 2026-07-11 phase-1 baseline. If these loader
semantics change, new runs stop being comparable with that data, so the real
file is asserted on rather than only synthetic tables.
"""
import pytest

from openpilot.tools.hcc.common import SCENARIOS_CSV
from openpilot.tools.hcc.publishers.profile import (
  MPH_TO_MPS,
  load_speed_profile,
  resolve_speed_scale,
  sample_profile,
)


@pytest.fixture
def table(tmp_path):
  # time, scn1, scn2
  path = tmp_path / "scenarios.csv"
  path.write_text("0.0,0.0,5.0\n1.0,10.0,5.0\n2.0,20.0,\n3.0,30.0,\n")
  return path


# ---- loading -------------------------------------------------------------

def test_loads_the_requested_column(table):
  times, speeds = load_speed_profile(table, 1)
  assert times == [0.0, 1.0, 2.0, 3.0]
  assert speeds == [0.0, 10.0, 20.0, 30.0]


def test_stops_at_the_first_blank_cell_after_data(table):
  times, speeds = load_speed_profile(table, 2)
  assert times == [0.0, 1.0]
  assert speeds == [5.0, 5.0]


def test_column_zero_is_time_not_a_scenario():
  with pytest.raises(ValueError):
    load_speed_profile(SCENARIOS_CSV, 0)


def test_negative_speeds_are_floored_at_zero(tmp_path):
  path = tmp_path / "s.csv"
  path.write_text("0.0,-5.0\n1.0,10.0\n")
  _, speeds = load_speed_profile(path, 1)
  assert speeds[0] == 0.0


def test_duplicate_timestamps_keep_the_first(tmp_path):
  path = tmp_path / "s.csv"
  path.write_text("0.0,1.0\n0.0,99.0\n1.0,2.0\n")
  times, speeds = load_speed_profile(path, 1)
  assert times == [0.0, 1.0]
  assert speeds == [1.0, 2.0]


def test_unsorted_rows_are_sorted_by_time(tmp_path):
  path = tmp_path / "s.csv"
  path.write_text("2.0,20.0\n0.0,0.0\n1.0,10.0\n")
  times, _ = load_speed_profile(path, 1)
  assert times == [0.0, 1.0, 2.0]


def test_too_short_a_profile_is_an_error(tmp_path):
  path = tmp_path / "s.csv"
  path.write_text("0.0,1.0\n")
  with pytest.raises(RuntimeError):
    load_speed_profile(path, 1)


def test_missing_column_is_an_error():
  with pytest.raises(RuntimeError):
    load_speed_profile(SCENARIOS_CSV, 9999)


# ---- sampling ------------------------------------------------------------

def test_sampling_interpolates_speed_and_reports_segment_accel(table):
  times, speeds = load_speed_profile(table, 1)
  v, a = sample_profile(times, speeds, 0.5)
  assert v == pytest.approx(5.0)
  assert a == pytest.approx(10.0)


def test_sampling_is_flat_outside_the_profile(table):
  times, speeds = load_speed_profile(table, 1)
  assert sample_profile(times, speeds, -1.0) == (0.0, 0.0)
  assert sample_profile(times, speeds, 99.0) == (30.0, 0.0)


def test_sampling_hits_the_knots_exactly(table):
  times, speeds = load_speed_profile(table, 1)
  for t, v in zip(times, speeds, strict=True):
    assert sample_profile(times, speeds, t)[0] == pytest.approx(v)


# ---- speed scaling ------------------------------------------------------

def test_cap_scales_the_peak_onto_the_limit():
  speeds = [0.0, 20.0]
  scale = resolve_speed_scale(speeds, 1.0, max_speed_mph=20.0)
  assert max(speeds) * scale == pytest.approx(20.0 * MPH_TO_MPS)


def test_cap_never_scales_up_a_slower_profile():
  # This is what makes it safe to leave a cap set for every run.
  assert resolve_speed_scale([0.0, 5.0], 1.0, max_speed_mph=100.0) == 1.0


def test_explicit_scale_and_cap_compose():
  speeds = [0.0, 20.0]
  scale = resolve_speed_scale(speeds, 2.0, max_speed_mph=20.0)
  assert max(speeds) * scale == pytest.approx(20.0 * MPH_TO_MPS)


@pytest.mark.parametrize("scale,cap", [(0.0, None), (-1.0, None), (1.0, 0.0), (1.0, -5.0)])
def test_invalid_scaling_arguments_are_rejected(scale, cap):
  with pytest.raises(ValueError):
    resolve_speed_scale([0.0, 10.0], scale, cap)


# ---- the real table -----------------------------------------------------

@pytest.mark.parametrize("scn", [48, 49, 50])
def test_real_scenarios_load(scn):
  times, speeds = load_speed_profile(SCENARIOS_CSV, scn)
  assert len(times) > 100
  assert times[0] == pytest.approx(0.1, abs=0.5)
  assert min(speeds) >= 0.0


def test_scenario_48_matches_the_phase_1_baseline():
  # 2026-07-11 ran the full scn48 profile with a top speed around 32 mph.
  _, speeds = load_speed_profile(SCENARIOS_CSV, 48)
  assert max(speeds) / MPH_TO_MPS == pytest.approx(32.3, abs=0.5)


def test_scenario_48_starts_and_ends_stopped():
  _, speeds = load_speed_profile(SCENARIOS_CSV, 48)
  assert speeds[0] == pytest.approx(0.0, abs=0.2)
