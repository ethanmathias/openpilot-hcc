"""Axis identification for the pedal rig.

A 3-pedal rig exposes several axes in no documented order. Getting the mapping
wrong is not visible afterwards: a swapped throttle and brake still produces a
plausible trace, and the run is only found to be useless when the acceleration
goes the wrong way. So the mapping is measured.

The fakes here reproduce the real MOZA SRP: every axis rests at -1.0 and
travels toward +1.0, and the load-cell brake does not reach full scale.
"""
from __future__ import annotations

import pytest

from openpilot.selfdrive.controls.lib.hcc.blend import pedal_to_accel
from openpilot.tools.hcc.publishers.pedal_source import AxisCalibration, identify_axes

REST = -1.0
FULL = 1.0
BRAKE_HELD = 0.66        # what a load cell actually reads under normal foot force

FAST = dict(stable_s=0.05, timeout_s=3.0, settle_s=0.05)


class FakeRig:
  """Presses the pedal named in the prompt, then releases on its own.

  Releasing automatically is what lets the next step's wait-for-rest succeed,
  and is also how a human behaves.
  """

  device_name = "Fake SRP"

  def __init__(self, num_axes=4, throttle=0, brake=1, clutch=None, release_after=30):
    self.num_axes = num_axes
    self._throttle, self._brake, self._clutch = throttle, brake, clutch
    self._release_after = release_after
    self._held: dict[int, float] = {}
    self._reads = 0

  def _press(self, mapping: dict[int, float]) -> None:
    self._held = mapping
    self._reads = 0

  def all_axes(self) -> list[float]:
    self._reads += 1
    if self._reads > self._release_after:
      self._held = {}
    return [self._held.get(i, REST) for i in range(self.num_axes)]

  def prompt(self, line: str) -> None:
    text = line.upper()
    if "HOLD THE THROTTLE" in text:
      held = {self._throttle: FULL}
      if self._clutch is not None:
        held[self._clutch] = FULL
      self._press(held)
    elif "HOLD THE BRAKE" in text:
      self._press({self._brake: BRAKE_HELD})


def _run(rig: FakeRig, extra_prompt=None) -> dict:
  def prompt(line: str) -> None:
    if extra_prompt is not None:
      extra_prompt(line)
    rig.prompt(line)
  return identify_axes(rig, prompt=prompt, **FAST)


# ---- the mapping is found, whatever the order ----------------------------

@pytest.mark.parametrize("throttle,brake", [(0, 1), (1, 0), (1, 2), (3, 0), (0, 3)])
def test_finds_each_pedal(throttle, brake):
  result = _run(FakeRig(throttle=throttle, brake=brake))
  assert result["throttle"] == throttle
  assert result["brake"] == brake


def test_rest_is_measured_not_assumed():
  # The MOZA rests at -1.0. Assuming 0.0 would read a released pedal as half
  # travel, and the published throttle would never fall below 0.5.
  result = _run(FakeRig())
  assert all(v == pytest.approx(REST, abs=0.01) for v in result["rest"])


def test_load_cell_brake_short_of_full_travel_is_still_identified():
  result = _run(FakeRig(throttle=0, brake=1))
  assert result["brake"] == 1
  assert result["held"]["brake"] == pytest.approx(BRAKE_HELD, abs=0.01)


def test_simultaneous_clutch_does_not_steal_the_result():
  rig = FakeRig(throttle=0, brake=1, clutch=3)
  result = _run(rig)
  assert result["throttle"] == 0
  assert result["deviations"]["throttle"][3] > 0.3     # clutch seen and reported
  assert result["brake"] == 1


# ---- the regression this rewrite exists for -----------------------------

class SlowReleaseRig(FakeRig):
  """The previous pedal decays back to rest instead of snapping.

  The first version of identify_axes sampled a fixed window starting the
  instant the prompt printed, so it caught the throttle on its way down and
  resolved BOTH pedals onto the throttle's axis. Observed on real hardware
  2026-11-17: brake deviations were {0: 2.0, 1: 1.664}, and axis 0 won.
  """

  DECAY_READS = 25

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._decay = 0

  def all_axes(self) -> list[float]:
    self._reads += 1
    if self._reads > self._release_after and self._held:
      if self._throttle in self._held:
        self._decay = self.DECAY_READS      # foot comes off slowly
      self._held = {}

    values = []
    for i in range(self.num_axes):
      if i in self._held:
        values.append(self._held[i])
      elif i == self._throttle and self._decay > 0:
        values.append(REST + (FULL - REST) * (self._decay / self.DECAY_READS))
      else:
        values.append(REST)
    if self._decay > 0:
      self._decay -= 1
    return values


def test_a_slowly_released_throttle_is_not_mistaken_for_the_brake():
  rig = SlowReleaseRig(throttle=0, brake=1)
  result = _run(rig)
  assert result["throttle"] == 0
  assert result["brake"] == 1, "the decaying throttle was mis-read as the brake"
  assert result["throttle"] != result["brake"]


# ---- failure reporting ---------------------------------------------------

def test_reports_nothing_held():
  lines: list[str] = []

  class Dead(FakeRig):
    def prompt(self, line: str) -> None:
      pass   # operator never pressed anything

  result = identify_axes(Dead(), prompt=lines.append, stable_s=0.05, timeout_s=0.3, settle_s=0.05)
  assert result["throttle"] is None and result["brake"] is None
  assert any("was the throttle pressed" in line.lower() for line in lines)


def test_warns_when_both_resolve_to_one_axis():
  lines: list[str] = []
  rig = FakeRig(throttle=1, brake=1)
  _run(rig, extra_prompt=lines.append)
  assert any("SAME axis" in line for line in lines)


# ---- the mapping means something downstream ------------------------------

def test_identified_calibration_spans_rest_to_travel():
  result = _run(FakeRig())
  cal = AxisCalibration(rest=result["rest"][result["throttle"]], full=FULL)
  assert cal.normalize(REST) == pytest.approx(0.0, abs=0.01)
  assert cal.normalize(FULL) == pytest.approx(1.0, abs=0.01)


def test_a_swapped_mapping_inverts_the_commanded_accel():
  """Why this is measured rather than guessed."""
  cal = AxisCalibration(rest=REST, full=FULL)
  braking = (cal.normalize(REST), cal.normalize(FULL))   # throttle rest, brake full
  assert pedal_to_accel(*braking) < 0.0
  assert pedal_to_accel(*reversed(braking)) > 0.0
