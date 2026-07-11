import numpy as np

from openpilot.selfdrive.controls.lib.hcc_v2v import V2VLeadSignal

# Feedforward lead-lag compensator F(s) = (tau*s + (1 - beta*th_bar)) / (th_bar*s + 1),
# matching the BeamNG reference (hccintegration/hCCC_controller_latest.py).  tau is
# the lead-vehicle actuation-lag time constant (numerator zero); th_bar is the
# nominal headway time (denominator pole).  th_bar=1.5 makes the feedforward
# consistent with the 1.5 s headway: DC gain = 1 - 0.65*1.5 = 0.025, i.e. the
# feedforward is essentially transient-only.
_FEEDFORWARD_TAU_S = 0.12
_FEEDFORWARD_TH_BAR_S = 1.5

# PID speed-tracking stage (BeamNG reference gains, tuned there at dt=0.1).
# K_I integrates error*dt, so it is dt-invariant as-is.  The reference derivative
# is PER-STEP (K_D * (e[k]-e[k-1]) with no /dt) at dt=0.1; the dt-invariant
# equivalent is a continuous gain of K_D * 0.1 = 0.02 s applied to de/dt.
_PID_KP = 0.35
_PID_KI = 0.05           # 1/s
_PID_KD_S = 0.02         # s (= reference K_D 0.2 per-step at its dt of 0.1 s)
# Anti-windup clamp (absent in the reference, required in a long-running real-car
# process): the integral's authority is capped at the full normalized output, so
# a saturated phase can never bank more than one full command of recovery lag.
_PID_I_LIMIT = 1.0 / _PID_KI
# Low-speed attenuation band from the reference: commands are scaled by
# v_des/10 when the desired speed is inside this band.
_PID_LOW_SPEED_LO_MPS = 0.5
_PID_LOW_SPEED_HI_MPS = 10.0

# The PID emits a normalized command in [-1, 1] (a pedal fraction in BeamNG).
# On the road, +/-1 maps to the reference design's own +/-3 m/s^2 authority
# envelope (its max_acceleration/max_deceleration constructor bounds); the
# planner's per-tick accel limits still clamp on top of this.
_OUTPUT_ACCEL_SCALE = 3.0


class PIDLongitudinal:
  """Speed-tracking PID from the BeamNG reference, made safe for controlsd.

  Differences from the reference implementation (behavior-preserving at the
  reference's dt, correct at any dt):
  - scalar running integral instead of an unbounded, re-summed error buffer;
  - integral clamped for anti-windup;
  - derivative computed per-second (see _PID_KD_S) instead of per-step;
  - first step after reset is pure P, matching the reference's len<2 branch.
  """

  def __init__(self, k_p: float, k_i: float, k_d_s: float, dt: float, i_limit: float):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d_s = k_d_s
    self._dt = dt
    self._i_limit = i_limit
    self._integral = 0.0
    self._prev_error = None

  def reset(self):
    self._integral = 0.0
    self._prev_error = None

  def run_step(self, target_speed: float, current_speed: float) -> float:
    error = target_speed - current_speed
    self._integral = float(np.clip(self._integral + error * self._dt, -self._i_limit, self._i_limit))

    if self._prev_error is not None:
      i_term = self._k_i * self._integral
      d_term = self._k_d_s * (error - self._prev_error) / self._dt
    else:
      i_term = 0.0
      d_term = 0.0
    self._prev_error = error

    accel_cmd = self._k_p * error + d_term + i_term

    if _PID_LOW_SPEED_LO_MPS < target_speed < _PID_LOW_SPEED_HI_MPS:
      accel_cmd = accel_cmd * target_speed / _PID_LOW_SPEED_HI_MPS
    return float(np.clip(accel_cmd, -1.0, 1.0))


class HCCC:
  """Human-in-the-Loop Cooperative Cruise Control (hCCC) for OpenPilot."""

  def __init__(self, dt=0.1, t_h=1.5, beta=0.65, max_deceleration=-3.0,
               max_acceleration=3.0):
    self._dt = dt
    self._t_h = t_h
    self._beta = beta
    self._max_decel = max_deceleration
    self._max_accl = max_acceleration

    # Bilinear (Tustin) discretization of the lead-lag feedforward F(s), computed
    # once in closed form so we avoid a scipy dependency in controlsd.  The
    # difference equation is y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1].
    self._ff_b0, self._ff_b1, self._ff_a1 = self._feedforward_coeffs()

    # Speed-tracking PID: the law produces a desired speed; the PID converts the
    # desired-speed error into a normalized accel command.
    self._pid = PIDLongitudinal(_PID_KP, _PID_KI, _PID_KD_S, dt, _PID_I_LIMIT)

    # Scalar state for the feedforward filter (avoids needing full history):
    # previous filter input (lead accel) and previous filter output.
    self._prev_lead_speed = None
    self._ff_prev_in = 0.0
    self._ff_prev_out = 0.0
    # Debug fields exported through the simulator logging path for comparing
    # live controller inputs/outputs against replay traces.
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_v_des = 0.0
    self.debug_pid_i = 0.0
    self.debug_output = 0.0

  def _reset_debug_state(self):
    self.debug_lead_speed = 0.0
    self.debug_lead_accel = 0.0
    self.debug_feedforward = 0.0
    self.debug_v_des = 0.0
    self.debug_pid_i = 0.0
    self.debug_output = 0.0

  def reset(self):
    self._prev_lead_speed = None
    self._ff_prev_in = 0.0
    self._ff_prev_out = 0.0
    self._pid.reset()
    self._reset_debug_state()

  def set_accel_limits(self, max_decel: float, max_accel: float):
    """Update the acceleration clamp bounds (called by longcontrol each tick)."""
    self._max_decel = max_decel
    self._max_accl = max_accel

  def _feedforward_coeffs(self):
    """Closed-form bilinear (Tustin) discretization of the lead-lag feedforward
    F(s) = (tau*s + c0) / (th_bar*s + 1), with c0 = 1 - beta*th_bar.

    Bilinear substitution s = (2/dt)*(1 - z^-1)/(1 + z^-1) and normalization by the
    leading denominator coefficient yields the difference equation
      y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1].
    This reproduces scipy.signal's TransferFunction(...).to_discrete(dt, 'tustin').
    """
    tau = _FEEDFORWARD_TAU_S
    th_bar = _FEEDFORWARD_TH_BAR_S
    c0 = 1.0 - self._beta * th_bar
    k = 2.0 / self._dt
    a0 = th_bar * k + 1.0
    b0 = (tau * k + c0) / a0
    b1 = (c0 - tau * k) / a0
    a1 = (1.0 - th_bar * k) / a0
    return b0, b1, a1

  def _feedforward_no_delay(self, a_lead: float) -> float:
    """Lead-lag feedforward filter (matches the BeamNG version's discretized F(s))."""
    out = self._ff_b0 * a_lead + self._ff_b1 * self._ff_prev_in - self._ff_a1 * self._ff_prev_out
    self._ff_prev_in = a_lead
    self._ff_prev_out = out
    return out

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

    # Cascade: the law produces a desired speed; the PID tracks it.  The
    # BeamNG reference's spacing term is intentionally absent (its gap input
    # does not exist on the V2V signal); this matches the reference's active
    # (non-spacing) variant.
    v_des = max(ego_speed + self._beta * speed_error + feedforward, 0.0)
    normalized_cmd = self._pid.run_step(v_des, ego_speed)
    accl_command = normalized_cmd * _OUTPUT_ACCEL_SCALE

    self.debug_lead_speed = float(lead_speed)
    self.debug_lead_accel = float(pre_accl_current)
    self.debug_feedforward = float(feedforward)
    self.debug_v_des = float(v_des)
    self.debug_pid_i = float(self._pid._integral)
    self.debug_output = float(accl_command)

    accl_command = np.clip(accl_command, self._max_decel, self._max_accl)
    return float(accl_command)
