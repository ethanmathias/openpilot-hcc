"""The human-in-the-loop term: converting pedal input into an accel offset.

The blend site itself is one line in longcontrol:

    output_accel = controller_accel + manual_accel

What this module decides is where manual_accel comes from and what happens
when that source goes quiet.

Two sources, in order of preference:

  1. The pedal rig, arriving over V2V as a V2VPedalSignal. This is the Fall-26
     configuration: the subject works a load-cell rig in the passenger
     footwell, their feet never touch the car's own pedals, and so the panda
     never sees an override.
  2. CarState.gas / CarState.brake, kept as a fallback for a platform that
     reports continuous pedals. On the 2026-07-11 Tucson runs this path
     produced exactly 0.0 in 100% of samples -- CarState.gas is deprecated out
     of the opendbc schema and Hyundai never populates CarState.brake -- which
     is why source 1 exists. Tesla does report continuous pedals
     (DI_accelPedalPos, IBST_sInputRodDriver) but openpilot discards the
     resolution, so this path is still 0.0 there today.

The failure behavior is the part that needed a decision rather than a port.
The lead stream degrades safely by construction: a stale lead means zero
command and the car coasts. A stale *pedal* stream does not have that
property. If the subject is holding the brake when the link drops, the naive
behavior is that their brake request vanishes and the hCCC command -- possibly
positive -- resumes alone. So on pedal staleness we zero the manual term AND
deny positive acceleration authority until the link is healthy again. It fails
in the direction of slowing down.
"""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import V2VPedalSignal

# Full travel on the rig contributes this much acceleration. These are the
# gains the BeamNG/sim work used; section 6 of the pedals-over-V2V design says
# to start here and retune from field data rather than guessing, since a
# 100 kg load cell against the panda's -3.5 m/s2 floor is a different feel.
ROAD_PEDAL_GAS_GAIN = 0.4
ROAD_PEDAL_BRAKE_GAIN = 1.2

SOURCE_NONE = "none"
SOURCE_PEDAL = "pedal"
SOURCE_CARSTATE = "carstate"


def normalize_pedal(value: float) -> float:
  """Accept either a fraction in [0, 1] or a percentage in (1, 100]."""
  v = float(value)
  if v > 1.0 and v <= 100.0:
    v *= 0.01
  return float(np.clip(v, 0.0, 1.0))


def pedal_to_accel(throttle: float, brake: float) -> float:
  """Signed accel offset from a throttle/brake pair, with the road gains."""
  net = float(np.clip(normalize_pedal(throttle) - normalize_pedal(brake), -1.0, 1.0))
  return net * (ROAD_PEDAL_GAS_GAIN if net >= 0.0 else ROAD_PEDAL_BRAKE_GAIN)


class ManualBlend:
  """Produces the manual accel term and the accel limits that go with it."""

  def __init__(self, pedal_enabled: bool = False):
    self.pedal_enabled = pedal_enabled
    self.manual_accel = 0.0
    self.source = SOURCE_NONE
    self.pedal_ok = False
    self.pedal_age_ms = float("inf")
    self.pedal_seq = -1
    self.authority_clamped = False

  def reset(self) -> None:
    self.manual_accel = 0.0
    self.source = SOURCE_NONE
    self.pedal_ok = False
    self.pedal_age_ms = float("inf")
    self.pedal_seq = -1
    self.authority_clamped = False

  def update(self, CS, pedal: V2VPedalSignal | None,
             accel_limits: tuple[float, float]) -> tuple[float, tuple[float, float]]:
    """Returns (manual_accel, accel_limits).

    accel_limits comes back with accel_max forced to 0.0 whenever the pedal
    source is enabled but its stream is not healthy.
    """
    accel_min, accel_max = float(accel_limits[0]), float(accel_limits[1])
    self.pedal_ok = bool(pedal is not None and pedal.status)
    self.pedal_age_ms = float("inf") if pedal is None else float(pedal.receive_age_ms)
    self.pedal_seq = -1 if pedal is None else int(pedal.seq)

    if self.pedal_enabled:
      if self.pedal_ok:
        assert pedal is not None
        self.manual_accel = pedal_to_accel(pedal.throttle, pedal.brake)
        self.source = SOURCE_PEDAL
        self.authority_clamped = False
      else:
        # Stale, crashed publisher, laptop asleep, or rig unplugged -- all
        # arrive here identically, which is why the publisher heartbeats at
        # 50 Hz regardless of pedal motion: silence is unambiguous.
        self.manual_accel = 0.0
        self.source = SOURCE_NONE
        self.authority_clamped = True
        accel_max = min(accel_max, 0.0)
    else:
      self.manual_accel = pedal_to_accel(getattr(CS, "gas", 0.0), getattr(CS, "brake", 0.0))
      self.source = SOURCE_CARSTATE
      self.authority_clamped = False

    return self.manual_accel, (accel_min, accel_max)
