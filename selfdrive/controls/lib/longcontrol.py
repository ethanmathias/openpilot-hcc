import numpy as np
import os
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.hccc_controller import hCCC
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
SIMULATION = os.environ.get("SIMULATION", "0") == "1"

# HCCC_CHANGE_NOTE: longcontrol uses BeamNG-style cooperative blending:
# controller contribution + signed manual pedal input, blended once in the shared
# longitudinal path so it works for both simulation and real-car execution.

def _normalize_pedal(value: float) -> float:
  v = float(value)
  if v > 1.0 and v <= 100.0:
    v *= 0.01
  return float(np.clip(v, 0.0, 1.0))


def _manual_longitudinal_input(CS) -> float:
  gas_raw = getattr(CS, "gas", 0.0)
  brake_raw = getattr(CS, "brake", 0.0)

  gas = _normalize_pedal(gas_raw)
  brake = _normalize_pedal(brake_raw)

  # Fallback for platforms that only expose boolean pressed flags.
  if getattr(CS, "gasPressed", False):
    gas = max(gas, 1.0)
  if getattr(CS, "brakePressed", False):
    brake = max(brake, 1.0)

  manual_cmd = float(np.clip(gas - brake, -1.0, 1.0))

  # NOTE: This maps manual pedal input into the simulator's accel-command space
  # so the existing bridge conversion reproduces BeamNG-like pedal authority.
  # This should be revisited for the real-car version, where actuators.accel is
  # not simply converted back into throttle/brake with the simulator scaling.
  if manual_cmd >= 0.0:
    return manual_cmd * 1.4
  return manual_cmd * 4.0


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.last_output_accel = 0.0
    self.params = Params()
    self.use_hccc = False
    self.hccc = None
    self._refresh_hccc(force_reset=True)

  def reset(self):
    self._refresh_hccc(force_reset=True)

  def _hccc_enabled(self):
    # HCCC_CHANGE_NOTE: enable from CarParams override or persistent EnableHCCC toggle.
    cp_flag = getattr(self.CP, 'enableHCCC', None)
    if cp_flag is not None:
      return cp_flag
    if not self.params.check_key("EnableHCCC"):
      return SIMULATION
    return self.params.get_bool("EnableHCCC")

  def _refresh_hccc(self, force_reset=False):
    enabled = self._hccc_enabled()
    if not enabled:
      self.hccc = None
    elif self.hccc is None or force_reset:
      self.hccc = hCCC(dt=DT_CTRL, max_deceleration=self.CP.stopAccel, max_acceleration=max(self.CP.startAccel, 1.6))
    self.use_hccc = enabled

  def update(self, active, CS, a_target, should_stop, accel_limits, lead=None, lead_time_ns=None):
    """Update longitudinal control. This updates the state machine and runs hCCC."""
    self._refresh_hccc()
    accel_min, accel_max = accel_limits

    controller_accel = 0.0
    hccc_output = None
    if self.use_hccc and self.hccc is not None:
      self.hccc._max_decel = accel_min
      self.hccc._max_accl = accel_max
      hccc_output = self.hccc.run_step(CS, lead, lead_time_ns=lead_time_ns)
      if hccc_output is not None:
        controller_accel = hccc_output

    # HCCC_CHANGE_NOTE: BeamNG-style cooperative input is a single signed manual command.
    manual_accel = _manual_longitudinal_input(CS)

    if hccc_output is not None:
      should_stop = False

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if active and hccc_output is not None:
      self.long_control_state = LongCtrlState.pid
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      output_accel = controller_accel + manual_accel

    self.last_output_accel = np.clip(output_accel, accel_min, accel_max)
    return self.last_output_accel
