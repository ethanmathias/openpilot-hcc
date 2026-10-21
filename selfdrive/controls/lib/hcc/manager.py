"""The single object longcontrol talks to.

On hcc-ego this logic lived inside LongControl: the V2V mode state machine, the
hCCC call, the manual blend and seven debug fields were interleaved with
upstream's PID path in one 280-line file, and reading it required knowing which
lines were ours. HCCManager owns all of it, so the seam in longcontrol.py is a
constructor, one update() call and one branch.

Nothing here activates on its own. With no params set, enabled is False and
LongControl behaves exactly like upstream.
"""
from __future__ import annotations

from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.hcc.blend import ManualBlend
from openpilot.selfdrive.controls.lib.hcc.config import HCCSettings, load_pedal_config, load_v2v_config
from openpilot.selfdrive.controls.lib.hcc.controller import HCCC
from openpilot.selfdrive.controls.lib.hcc.state import (
  MODE_OFF,
  HCCResult,
  determine_mode,
)
from openpilot.selfdrive.controls.lib.hcc.transport.endpoints import V2VLeadSubscriber, V2VPedalSubscriber
from openpilot.selfdrive.controls.lib.hcc.transport.protocol import (
  ROLE_EGO,
  V2VLeadSignal,
  V2VPedalSignal,
)

class HCCManager:
  def __init__(self, CP, params=None, env=None):
    self.CP = CP
    self.settings = HCCSettings(params=params, env=env, CP=CP)

    self.enabled = False
    self.hccc: HCCC | None = None

    self.lead_config = load_v2v_config(ROLE_EGO, params=params, env=env,
                                       default_enabled=False, default_device_id="hcc-ego")
    self.lead_sub = V2VLeadSubscriber(self.lead_config) if self.lead_config.enabled else None
    if self.lead_sub is not None:
      self.lead_sub.start()

    self.pedal_config = load_pedal_config(params=params, env=env)
    self.pedal_sub = V2VPedalSubscriber(self.pedal_config) if self.pedal_config.enabled else None
    if self.pedal_sub is not None:
      self.pedal_sub.start()

    self.blend = ManualBlend(pedal_enabled=self.pedal_config.enabled)

    self.lead_signal = V2VLeadSignal(status=False)
    self.pedal_signal = V2VPedalSignal(status=False)
    self.mode = MODE_OFF
    self._last_logged_mode: str | None = None
    self.result = HCCResult()
    self.refresh(force_reset=True)

  # ---- configuration -------------------------------------------------------

  def refresh(self, force_reset: bool = False) -> None:
    """Re-read the master toggle and (re)build the controller if needed."""
    enabled = self.settings.hccc_enabled()
    if not enabled:
      self.hccc = None
    elif self.hccc is None or force_reset:
      self.hccc = HCCC(dt=DT_CTRL,
                       max_deceleration=self.CP.stopAccel,
                       max_acceleration=max(self.CP.startAccel, 1.6))
    self.enabled = enabled

  def reset(self) -> None:
    # Called from longcontrol on every off/stopping/starting tick, so this
    # zeroes controller state in place rather than rebuilding HCCC at 100 Hz.
    self.blend.reset()
    self.refresh()
    if self.hccc is not None:
      self.hccc.reset()

  def stop(self) -> None:
    """Release both sockets. Only used by tests and tooling; controlsd lives
    for the life of the process."""
    for sub in (self.lead_sub, self.pedal_sub):
      if sub is not None:
        sub.stop()

  # ---- link state ----------------------------------------------------------

  def v2v_enabled(self) -> bool:
    return self.lead_sub is not None and self.lead_config.enabled

  def v2v_only(self) -> bool:
    return self.settings.v2v_only()

  def pedal_link_enabled(self) -> bool:
    return self.pedal_sub is not None and self.pedal_config.enabled

  def lead_snapshot(self) -> V2VLeadSignal:
    self.lead_signal = self.lead_sub.snapshot() if self.v2v_enabled() else V2VLeadSignal(status=False)
    return self.lead_signal

  def pedal_snapshot(self) -> V2VPedalSignal:
    self.pedal_signal = self.pedal_sub.snapshot() if self.pedal_link_enabled() else V2VPedalSignal(status=False)
    return self.pedal_signal

  def longitudinal_ok(self, lead: V2VLeadSignal | None = None) -> bool:
    """Whether the V2V input is healthy enough to allow longitudinal control.

    Always True when the link is not in use, so this can be ANDed into
    CC.longActive unconditionally.
    """
    if not self.v2v_enabled():
      return True
    signal = self.lead_snapshot() if lead is None else lead
    self.lead_signal = signal
    return bool(signal.status)

  def transport_ok(self) -> bool:
    if self.lead_sub is None:
      return True
    return bool(self.lead_sub.transport_ok())

  def _determine_mode(self, active: bool, signal: V2VLeadSignal, transport_ok: bool) -> str:
    return determine_mode(self.enabled, self.v2v_enabled(), self.v2v_only(), active, signal, transport_ok)

  def _set_mode(self, mode: str, signal: V2VLeadSignal, transport_ok: bool) -> None:
    self.mode = mode
    if self._last_logged_mode != mode:
      cloudlog.info(
        "hcc_mode_transition mode=%s transport_ok=%s lead_status=%s lead_age_ms=%.2f lead_seq=%d "
        "pedal_status=%s pedal_age_ms=%.2f",
        mode, transport_ok, signal.status, float(signal.receive_age_ms), int(signal.seq),
        self.pedal_signal.status, float(self.pedal_signal.receive_age_ms),
      )
      self._last_logged_mode = mode

  # ---- per-tick ------------------------------------------------------------

  def update(self, active: bool, CS, accel_limits: tuple[float, float],
             lead=None, v2v_lead: V2VLeadSignal | None = None) -> HCCResult:
    self.refresh()
    accel_min, accel_max = float(accel_limits[0]), float(accel_limits[1])

    lead_signal = self.lead_signal if v2v_lead is None else v2v_lead
    transport_ok = True
    if self.v2v_enabled():
      lead_signal = self.lead_snapshot() if v2v_lead is None else v2v_lead
      self.lead_signal = lead_signal
      transport_ok = self.transport_ok()

    pedal_signal = self.pedal_snapshot()

    controller_accel = 0.0
    controller_active = False
    if self.enabled and self.hccc is not None:
      self.hccc.set_accel_limits(accel_min, accel_max)
      radar_lead = None if self.v2v_only() else lead
      v2v_input = lead_signal if self.v2v_enabled() else None
      hccc_output = self.hccc.run_step(CS, radar_lead, v2v_lead=v2v_input)
      if hccc_output is not None:
        controller_accel = float(hccc_output)
        controller_active = True

    manual_accel, clamped_limits = self.blend.update(CS, pedal_signal, (accel_min, accel_max))

    self._set_mode(self._determine_mode(active, lead_signal, transport_ok), lead_signal, transport_ok)

    self.result = HCCResult(
      enabled=self.enabled,
      controller_active=controller_active,
      controller_accel=controller_accel,
      manual_accel=manual_accel,
      accel_limits=clamped_limits,
      v2v_only=self.v2v_only(),
      v2v_lead=lead_signal,
      mode=self.mode,
    )
    return self.result

  # ---- telemetry -----------------------------------------------------------

  def debug_fields(self) -> dict[str, float]:
    """Values controlsd republishes through controlsState for the monitor."""
    hccc = self.hccc
    return {
      "hccc_accel": self.result.controller_accel,
      "manual_accel": self.result.manual_accel,
      "output_accel": self.result.output_accel,
      "lead_speed": float(getattr(hccc, "debug_lead_speed", 0.0)),
      "lead_accel": float(getattr(hccc, "debug_lead_accel", 0.0)),
      "feedforward": float(getattr(hccc, "debug_feedforward", 0.0)),
      "pedal_age_ms": self.blend.pedal_age_ms,
    }
