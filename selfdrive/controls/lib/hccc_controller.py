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
    self._prev_lead_time_ns = None
    self._prev_lead_accel = 0.0
    self.ff_y_prev = 0.0

  def feedforward_no_delay(self, a_lead):
    """Apply the same feedforward filter used in the BeamNG version."""
    th_bar = 1.0
    y = self.ff_y_prev + (self._dt / th_bar) * ((1 - th_bar * self._beta) * a_lead - self.ff_y_prev)
    self.ff_y_prev = y
    return y

  def run_step(self, CS, lead, lead_time_ns=None):
    if lead is None or not lead.status:
      self._prev_lead_speed = None
      self._prev_lead_time_ns = None
      self._prev_lead_accel = 0.0
      self.ff_y_prev = 0.0
      return None

    ego_speed = CS.vEgo
    lead_speed = float(getattr(lead, "vLead", ego_speed + lead.vRel))

    # Refresh the differentiated lead acceleration only when a new lead sample
    # arrives, using the actual elapsed time between radar updates.
    if (lead_time_ns is not None and self._prev_lead_speed is not None and
        self._prev_lead_time_ns is not None and lead_time_ns > self._prev_lead_time_ns):
      delta_t = max((lead_time_ns - self._prev_lead_time_ns) * 1e-9, 1e-3)
      pre_accl_current = (lead_speed - self._prev_lead_speed) / delta_t
    elif lead_time_ns is not None and lead_time_ns == self._prev_lead_time_ns:
      pre_accl_current = self._prev_lead_accel
    elif self._prev_lead_speed is not None:
      pre_accl_current = (lead_speed - self._prev_lead_speed) / self._dt
    else:
      pre_accl_current = 0.0
    self._prev_lead_speed = lead_speed
    self._prev_lead_time_ns = lead_time_ns
    self._prev_lead_accel = pre_accl_current

    feedforward_state = self.feedforward_no_delay(pre_accl_current)
    accl_command = (self._beta * (lead_speed - ego_speed) + feedforward_state) * 0.6

    accl_command = np.clip(accl_command, self._max_decel, self._max_accl)
    return float(accl_command)
