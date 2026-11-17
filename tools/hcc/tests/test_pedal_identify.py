"""Axis identification for the pedal rig.

A 3-pedal rig exposes several axes in no documented order. Getting the
mapping wrong is not obvious afterwards: a swapped throttle and brake still
produces a plausible-looking trace, and the run is only discovered to be
useless when the acceleration goes the wrong way. So the mapping is measured
rather than guessed, and this holds the measurement honest.

The real MOZA SRP rests at -1.0 on every axis and travels toward +1.0, which
is what the fake source below reproduces.
"""
from __future__ import annotations

import pytest

from openpilot.selfdrive.controls.lib.hcc.blend import pedal_to_accel
from openpilot.tools.hcc.publishers.pedal_source import AxisCalibration, identify_axes

REST = -1.0
PRESSED = 1.0


class FakeRig:
  """A rig whose pressed pedal is chosen by the prompt text, so the capture
  windows are deterministic and the test needs no sleeping."""

  device_name = "Fake SRP"

  def __init__(self, num_axes: int = 4, throttle: int = 1, brake: int = 2, clutch: int | None = None):
    self.num_axes = num_axes
    self._throttle = throttle
    self._brake = brake
    self._clutch = clutch
    self.pressed: set[int] = set()

  def all_axes(self) -> list[float]:
    return [PRESSED if i in self.pressed else REST for i in range(self.num_axes)]

  def prompt(self, line: str) -> None:
    text = line.upper()
    if "HOLD THE THROTTLE" in text:
      self.pressed = {self._throttle} | ({self._clutch} if self._clutch is not None else set())
    elif "HOLD THE BRAKE" in text:
      self.pressed = {self._brake}
    elif "FEET OFF" in text:
      self.pressed = set()


def _identify(rig: FakeRig) -> dict:
  return identify_axes(rig, hold_s=0.05, settle_s=0.05, prompt=rig.prompt)


# ---- the mapping is found, whatever the order ----------------------------

@pytest.mark.parametrize("throttle,brake", [(1, 2), (2, 1), (0, 3), (3, 0), (0, 1)])
def test_finds_the_pressed_axis(throttle, brake):
  result = _identify(FakeRig(throttle=throttle, brake=brake))
  assert result["throttle"] == throttle
  assert result["brake"] == brake


def test_rest_is_measured_not_assumed():
  # The MOZA rests at -1.0, not 0.0. Assuming zero would make a released
  # pedal read as half travel.
  result = _identify(FakeRig())
  assert all(v == pytest.approx(REST) for v in result["rest"])


def test_reports_every_axis_that_moved():
  # A 3-pedal rig can register the clutch at the same time; the operator
  # needs to see that rather than silently accept the wrong axis.
  rig = FakeRig(throttle=1, brake=2, clutch=3)
  result = _identify(rig)
  assert set(result["moved"]["throttle"]) == {1, 3}
  assert set(result["moved"]["brake"]) == {2}


def test_warns_when_throttle_and_brake_collide(capsys):
  lines: list[str] = []
  rig = FakeRig(throttle=1, brake=1)
  identify_axes(rig, hold_s=0.05, settle_s=0.05,
                prompt=lambda s: (lines.append(s), rig.prompt(s))[0])
  assert any("SAME axis" in line for line in lines)


def test_warns_when_nothing_moved():
  lines: list[str] = []

  class Dead(FakeRig):
    def prompt(self, line: str) -> None:
      self.pressed = set()   # operator never pressed anything

  rig = Dead()
  identify_axes(rig, hold_s=0.05, settle_s=0.05,
                prompt=lambda s: (lines.append(s), rig.prompt(s))[0])
  assert any("WARNING" in line and "pressed" in line for line in lines)


# ---- the mapping actually means something downstream ---------------------

def test_identified_calibration_maps_rest_to_zero_and_travel_to_one():
  result = _identify(FakeRig())
  cal = AxisCalibration(rest=result["rest"][result["throttle"]], full=PRESSED)
  assert cal.normalize(REST) == pytest.approx(0.0)
  assert cal.normalize(PRESSED) == pytest.approx(1.0)
  assert cal.normalize(0.0) == pytest.approx(0.5)


def test_a_swapped_mapping_inverts_the_commanded_accel():
  """Why this is measured rather than guessed.

  With throttle and brake exchanged, a full brake press commands positive
  acceleration. Nothing downstream can detect that.
  """
  cal = AxisCalibration(rest=REST, full=PRESSED)
  braking = (cal.normalize(REST), cal.normalize(PRESSED))      # throttle rest, brake full
  assert pedal_to_accel(*braking) < 0.0
  assert pedal_to_accel(*reversed(braking)) > 0.0
