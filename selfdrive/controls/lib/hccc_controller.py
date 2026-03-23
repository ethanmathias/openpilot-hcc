import numpy as np

# HCCC_CHANGE_NOTE: standalone hCCC implementation used by longcontrol and dedicated tests.

class hCCC:
  """Human-in-the-Loop Cooperative Cruise Control (hCCC) for OpenPilot."""

  def __init__(self, dt=0.1, t_h=1.5, beta=0.65, max_deceleration=-3.0,
               max_acceleration=3.0, jam=0.0):
    self._dt = dt
    self._t_h = t_h
    self._beta = beta
    self._max_decel = max_deceleration
    self._max_accl = max_acceleration
    self._jam = jam

    # Keep only scalar state needed for the controller update. So that infinite memory is not needed for the feedforward filter.
    self._prev_lead_speed = None
    self.ff_y_prev = 0.0
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_output = 0.0

  def reset(self):
    self._prev_lead_speed = None
    self.ff_y_prev = 0.0
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_output = 0.0

  def feedforward_no_delay(self, a_lead):
    """Apply the same feedforward filter used in the BeamNG version."""
    th_bar = 1.0
    y = self.ff_y_prev + (self._dt / th_bar) * ((1 - th_bar * self._beta) * a_lead - self.ff_y_prev)
    self.ff_y_prev = y
    return y

  # BeamNG parity: derive lead acceleration from lead speed history instead of radar accel.
  def run_step(self, CS, lead):
    if lead is None or not lead.status:
      self.reset()
      return None

    ego_speed = CS.vEgo
    lead_speed = ego_speed + lead.vRel

    if self._prev_lead_speed is not None and np.isfinite(self._prev_lead_speed):
      pre_accl_current = (lead_speed - self._prev_lead_speed) / self._dt
    else:
      pre_accl_current = 0.0
    self._prev_lead_speed = lead_speed

    feedforward_state = self.feedforward_no_delay(pre_accl_current)
    accl_command = self._beta * (lead_speed - ego_speed) + feedforward_state
    self.debug_lead_speed = float(lead_speed)
    self.debug_lead_accel = float(pre_accl_current)
    self.debug_feedforward = float(feedforward_state)
    self.debug_output = float(accl_command)

    return float(accl_command)
