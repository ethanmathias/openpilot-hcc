"""Mode and stop-state policy for the HCC longitudinal path.

Deliberately free of openpilot runtime imports (no cereal, no params, no
swaglog), so the decisions that matter can be unit-tested on any machine
without a built openpilot. manager.py does the plumbing; this file holds the
rules.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from openpilot.selfdrive.controls.lib.hcc.transport.protocol import V2VLeadSignal

MODE_OFF = "off"
MODE_MANUAL_ONLY = "manual_only"
MODE_WAITING = "v2v_waiting"
MODE_ACTIVE = "v2v_active"
MODE_FAULT_STALE = "v2v_fault_stale"
MODE_FAULT_TRANSPORT = "v2v_fault_transport"

# Below this V2V target speed the lead counts as stopped, so the standard
# stopping/starting state machine runs instead of the controller commanding
# through a stop. Stopping sends the car its stopReq, and leaving stopping
# performs the standstill release a drivetrain needs before it will accept
# positive accel again after a full stop.
#
# Field bug 5 (2026-07-11, Tucson): the HKG drivetrain latched its standstill
# hold while engaged and ignored positive accel, because the forced-pid path
# used then never performed that handshake. Running the real state machine is
# the general fix rather than a Hyundai workaround, so it is kept -- but the
# Tesla's standstill behavior is UNVERIFIED and is a thing to measure before
# trusting a stop-and-go run.
V2V_STOP_SPEED_MPS = 0.5


def determine_mode(hccc_enabled: bool, v2v_enabled: bool, v2v_only: bool,
                   active: bool, signal: V2VLeadSignal, transport_ok: bool) -> str:
  """Which mode the V2V longitudinal path is in, for logging and the monitor."""
  if not hccc_enabled:
    return MODE_OFF
  if not v2v_only or not v2v_enabled:
    return MODE_MANUAL_ONLY
  if not transport_ok:
    return MODE_FAULT_TRANSPORT
  if signal.status and active:
    return MODE_ACTIVE
  if signal.seq < 0 or signal.status:
    # Nothing received yet, or healthy but not engaged.
    return MODE_WAITING
  return MODE_FAULT_STALE


@dataclass
class HCCResult:
  """What longcontrol needs from one HCC tick."""
  enabled: bool = False
  controller_active: bool = False
  controller_accel: float = 0.0
  manual_accel: float = 0.0
  accel_limits: tuple[float, float] = (0.0, 0.0)
  v2v_only: bool = False
  v2v_lead: V2VLeadSignal = field(default_factory=lambda: V2VLeadSignal(status=False))
  mode: str = MODE_OFF

  @property
  def output_accel(self) -> float:
    return self.controller_accel + self.manual_accel

  def resolve_should_stop(self, upstream_should_stop: bool, v_ego: float, v_ego_stopping: float) -> bool:
    """The should_stop the long state machine should use this tick."""
    if not self.v2v_only:
      # Not V2V-only: a live controller command means we are not stopping.
      return False if self.controller_active else upstream_should_stop
    v_target = float(self.v2v_lead.lead_speed_mps) if self.controller_active else 0.0
    return (v_target < V2V_STOP_SPEED_MPS) and (v_ego < v_ego_stopping)
