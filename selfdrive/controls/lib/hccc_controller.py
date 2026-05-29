import numpy as np

from openpilot.selfdrive.controls.lib.hcc_v2v import V2VLeadSignal

# Final acceleration command is scaled by this factor after the beta * speed_error +
# feedforward computation.  Empirically tuned to avoid overshoot in the sim.
_ACCEL_OUTPUT_SCALE = 0.6

# Time constant (seconds) for the first-order feedforward low-pass filter.
_FEEDFORWARD_TIME_CONSTANT_S = 1.0


class HCCC:
  """Human-in-the-Loop Cooperative Cruise Control (hCCC) for OpenPilot."""

  def __init__(self, dt=0.1, t_h=1.5, beta=0.65, max_deceleration=-3.0,
               max_acceleration=3.0):
    self._dt = dt
    self._t_h = t_h
    self._beta = beta
    self._max_decel = max_deceleration
    self._max_accl = max_acceleration

    # Scalar state for the feedforward filter (avoids needing full history).
    self._prev_lead_speed = None
    self._feedforward_state = 0.0
    # Debug fields exported through the simulator logging path for comparing
    # live controller inputs/outputs against replay traces.
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_output = 0.0

  def _reset_debug_state(self):
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_output = 0.0

  def reset(self):
    self._prev_lead_speed = None
    self._feedforward_state = 0.0
    self._reset_debug_state()

  def set_accel_limits(self, max_decel: float, max_accel: float):
    """Update the acceleration clamp bounds (called by longcontrol each tick)."""
    self._max_decel = max_decel
    self._max_accl = max_accel

  def _feedforward_no_delay(self, a_lead: float) -> float:
    """First-order low-pass feedforward filter (matches the BeamNG version)."""
    tau = _FEEDFORWARD_TIME_CONSTANT_S
    self._feedforward_state += (self._dt / tau) * (
      (1 - tau * self._beta) * a_lead - self._feedforward_state
    )
    return self._feedforward_state

  # Keep a small amount of internal state so replay logging can reconstruct the
  # lead-speed and feedforward terms that drive the HC3 command.
  def run_step(self, CS, lead, v2v_lead: V2VLeadSignal | None = None):
    ego_speed = CS.vEgo

    if v2v_lead is not None:
      if not v2v_lead.status:
        self.reset()
        return None
      lead_speed = float(v2v_lead.lead_speed_mps)
      pre_accl_current = float(v2v_lead.lead_accel_mps2)
    else:
      if lead is None or not lead.status:
        self.reset()
        return None

      lead_speed = ego_speed + lead.vRel

      radar_lead_accel = getattr(lead, "aLeadK", None)
      if radar_lead_accel is not None and np.isfinite(radar_lead_accel):
        pre_accl_current = float(radar_lead_accel)
      elif self._prev_lead_speed is not None:
        pre_accl_current = (lead_speed - self._prev_lead_speed) / self._dt
      else:
        pre_accl_current = 0.0

    self._prev_lead_speed = lead_speed

    feedforward = self._feedforward_no_delay(pre_accl_current)
    speed_error = lead_speed - ego_speed
    accl_command = (self._beta * speed_error + feedforward) * _ACCEL_OUTPUT_SCALE
    self.debug_lead_speed = float(lead_speed)
    self.debug_lead_accel = float(pre_accl_current)
    self.debug_feedforward = float(feedforward)
    self.debug_output = float(accl_command)

    accl_command = np.clip(accl_command, self._max_decel, self._max_accl)
    return float(accl_command)
