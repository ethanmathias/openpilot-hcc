"""Reading and sampling a scenario speed profile.

Kept separate from the publisher so the arithmetic that decides what the ego
is asked to follow can be tested without opening a socket.

config/scenarios.csv is headerless: column 0 is time in seconds, column N is
scenario N's speed in m/s. Scenario 48 at 35 mph is the phase-1 baseline, so
these semantics match the simulator's loader exactly -- changing them would
make new runs incomparable with 2026-07-11.
"""
from __future__ import annotations

import csv
from pathlib import Path

MPH_TO_MPS = 0.44704
SENT_CSV_COLUMNS = ["wall_time_us", "profile_time_s", "seq", "v_lead_mps", "a_lead_mps2"]


def load_speed_profile(csv_path: str | Path, scenario_index: int) -> tuple[list[float], list[float]]:
  """Read (time_s, speed_mps) samples for one scenario column.

  Reading stops at the first blank or invalid cell after data has started, so a
  short scenario in a wide table ends where its data ends.
  """
  if scenario_index < 1:
    raise ValueError(f"scenario index must be >= 1, got {scenario_index}")

  times: list[float] = []
  speeds: list[float] = []
  with open(csv_path, newline="") as f:
    for row in csv.reader(f):
      if not row or not row[0].strip():
        continue
      try:
        t = float(row[0].strip())
      except ValueError:
        continue
      if scenario_index >= len(row):
        if speeds:
          break
        continue
      cell = row[scenario_index].strip()
      if not cell:
        if speeds:
          break
        continue
      try:
        v = max(float(cell), 0.0)
      except ValueError:
        if speeds:
          break
        continue
      times.append(t)
      speeds.append(v)

  if len(times) < 2:
    raise RuntimeError(f"scenario {scenario_index} has fewer than 2 samples in {csv_path}")

  # Sort by time and drop duplicate timestamps (keep the first occurrence).
  order = sorted(range(len(times)), key=lambda i: times[i])
  out_t: list[float] = []
  out_v: list[float] = []
  for i in order:
    if out_t and times[i] == out_t[-1]:
      continue
    out_t.append(times[i])
    out_v.append(speeds[i])
  if len(out_t) < 2:
    raise RuntimeError(f"scenario {scenario_index} needs at least two unique time samples in {csv_path}")
  return out_t, out_v


def sample_profile(times: list[float], speeds: list[float], t: float) -> tuple[float, float]:
  """(v, a) at profile time t: linear speed interpolation, per-segment accel."""
  if t <= times[0]:
    return speeds[0], 0.0
  if t >= times[-1]:
    return speeds[-1], 0.0

  lo, hi = 0, len(times) - 1
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if times[mid] <= t:
      lo = mid
    else:
      hi = mid
  dt = times[hi] - times[lo]
  a = (speeds[hi] - speeds[lo]) / dt if dt > 0.0 else 0.0
  v = speeds[lo] + a * (t - times[lo])
  return v, a


def resolve_speed_scale(speeds: list[float], speed_scale: float = 1.0,
                        max_speed_mph: float | None = None) -> float:
  """Combine an explicit scale with an optional top-speed cap into one factor.

  The cap scales the whole profile proportionally -- speeds AND the
  accelerations derived from them -- so the scaled peak lands exactly on the
  cap. It never scales UP a profile that is already slower, which is what makes
  it safe to leave a cap set for every run.
  """
  if speed_scale <= 0.0:
    raise ValueError(f"speed_scale must be > 0, got {speed_scale}")
  scale = float(speed_scale)
  if max_speed_mph is not None:
    if max_speed_mph <= 0.0:
      raise ValueError(f"max_speed_mph must be > 0, got {max_speed_mph}")
    peak = max(speeds) * scale
    cap_mps = max_speed_mph * MPH_TO_MPS
    if peak > cap_mps:
      scale *= cap_mps / peak
  return scale


def describe(times: list[float], speeds: list[float]) -> str:
  return (f"{len(times)} samples, {times[-1] - times[0]:.1f} s, "
          f"v in [{min(speeds):.2f}, {max(speeds):.2f}] m/s "
          f"(top {max(speeds) / MPH_TO_MPS:.1f} mph)")
